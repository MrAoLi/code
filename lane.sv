`timescale 1ns/1ps

module lane #(
    parameter int unsigned NrLanes         = 4,
    parameter int unsigned VLEN            = 4096,
    parameter int unsigned BytesPerLane    = 4096,
    parameter int unsigned MaxVLenPerLane  = VLEN/NrLanes,
    parameter int unsigned NumBanks        = 8,
    parameter int unsigned BankWidth       = 64,
    parameter int unsigned WordsPerBank    = 64,
    parameter int unsigned OpQueueDepth    = 8,

    parameter type global_vaddr_t = logic[11:0],
    parameter type lane_strb_t    = logic[7:0],

    parameter type pe_req_t = struct packed {
      global_vaddr_t vaddr;
      logic [31:0]   op_code;
      logic [63:0]   tf1;
      logic [63:0]   tf2;
      logic [63:0]   tf3;
      logic          wr_en;
      logic [MaxVLenPerLane-1:0] data;
      lane_strb_t    be;
    },

    parameter type pe_resp_t = struct packed {
      global_vaddr_t vaddr;
      logic [MaxVLenPerLane-1:0] data;
      logic         valid;
      logic         ready;
    }
  ) (
    input  logic                       clk_i,
    input  logic                       rst_ni,
    input  logic [$clog2(NrLanes)-1:0] lane_id_i,

    // 指令接口
    input  logic                       instr_valid_i,
    input  pe_req_t                    instr_i,
    output logic                       instr_ready_o,

    // 响应接口
    output logic                       resp_valid_o,
    output pe_resp_t                   resp_o,
    input  logic                       resp_ready_i,

    // VRF写回接口
    output logic                       vrf_wr_valid_o,
    output global_vaddr_t              vrf_wr_addr_o,
    output lane_strb_t                 vrf_wr_be_o,
    output logic [BankWidth-1:0]       vrf_wr_data_o,

    // 旋转因子接口
    input  logic [63:0]                tf1_i,
    input  logic [63:0]                tf2_i,
    input  logic [63:0]                tf3_i
  );

  // --------------------------------------------------
  // opcode 定义
  localparam logic [31:0]
             OPCODE_NOP   = 32'h0000_0000, // 空操作
             OPCODE_LD    = 32'h0000_0001, // 加载指令
             OPCODE_ST    = 32'h0000_0002, // 存储指令
             OPCODE_FFT   = 32'h0000_0003; // FFT指令

  // 操作类型枚举
  typedef enum logic [1:0] {
            OP_LOAD,
            OP_STORE,
            OP_FFT,
            OP_NOP
          } op_type_t;

  // 状态与握手信号
  logic seq_valid;
  pe_req_t seq_instr;
  logic seq_ready;
  logic seq_resp_ready;

  // opcode 译码信号
  logic ld_req, st_req, fft_req;
  op_type_t current_op_type;

  // FFT 相关信号
  logic fft_start, fft_busy, fft_valid, fft_ready;
  logic [63:0] Y0, Y1, Y2, Y3;
  logic [63:0] fft_result_Y0, fft_result_Y1, fft_result_Y2, fft_result_Y3;
  logic [255:0] fft_input_data;  // 4 * 64-bit for FFT
  logic [63:0] fft_X0, fft_X1, fft_X2, fft_X3;  // FFT输入寄存器
  
  // FFT状态管理
  typedef enum logic [2:0] {
    FFT_IDLE,
    FFT_READ_DATA,
    FFT_COMPUTING,
    FFT_WAIT_RESULT,
    FFT_DONE
  } fft_state_t;
  
  fft_state_t fft_state, fft_next_state;
  logic [2:0] fft_counter;  // FFT流水线计数器

  // VRF 相关信号
  logic [MaxVLenPerLane-1:0] vrf_read_data;
  logic vrf_read_valid;

  // Sequencer 实例化
  lane_sequencer_enhanced #(
                            .OpQueueDepth(OpQueueDepth),
                            .NrLanes(NrLanes),
                            .VLEN(VLEN),
                            .BytesPerLane(BytesPerLane),
                            .MaxVLenPerLane(MaxVLenPerLane),
                            .NumBanks(NumBanks),
                            .BankWidth(BankWidth),
                            .WordsPerBank(WordsPerBank),
                            .global_vaddr_t(global_vaddr_t),
                            .lane_strb_t(lane_strb_t),
                            .pe_req_t(pe_req_t)
                          ) u_sequencer (
                            .clk_i          (clk_i),
                            .rst_ni         (rst_ni),
                            .instr_valid_i  (instr_valid_i),
                            .instr_i        (instr_i),
                            .instr_ready_o  (instr_ready_o),
                            .decoded_valid_o(seq_valid),
                            .decoded_instr_o(seq_instr),
                            .decoded_ready_i(seq_ready),
                            .resp_valid_i   (resp_valid_o),
                            .resp_ready_o   (seq_resp_ready)
                          );

  // 译码逻辑
  always_comb
  begin
    ld_req  = seq_valid && (seq_instr.op_code == OPCODE_LD);
    st_req  = seq_valid && (seq_instr.op_code == OPCODE_ST);
    fft_req = seq_valid && (seq_instr.op_code == OPCODE_FFT);

    // 确定当前操作类型
    if (ld_req)
      current_op_type = OP_LOAD;
    else if (st_req)
      current_op_type = OP_STORE;
    else if (fft_req)
      current_op_type = OP_FFT;
    else
      current_op_type = OP_NOP;
  end

  // 地址解码
  logic [2:0] wr_bank_sel, rd_bank_sel;
  logic [5:0] wr_word_addr, rd_word_addr;
  
  assign wr_bank_sel = vrf_wr_addr_o[2:0];
  assign wr_word_addr = vrf_wr_addr_o[8:3];
  assign rd_bank_sel = seq_instr.vaddr[2:0];
  assign rd_word_addr = seq_instr.vaddr[8:3];

  // VRF Bank Array
  logic [NumBanks-1:0]                vrf_wr_en, vrf_rd_en;
  logic [NumBanks-1:0][BankWidth-1:0] vrf_rd_data;
  logic [NumBanks-1:0]                vrf_rd_valid;

  generate
    for (genvar i = 0; i < NumBanks; i++)
    begin : bank_gen
      vrf_bank #(.Depth(WordsPerBank), .DataWidth(BankWidth)) u_bank (
                 .clk_i      (clk_i),
                 .rst_ni     (rst_ni),
                 .wr_en_i    (vrf_wr_en[i]),
                 .wr_addr_i  (wr_word_addr),
                 .wr_data_i  (vrf_wr_data_o),
                 .wr_be_i    (vrf_wr_be_o),
                 .rd_en_i    (vrf_rd_en[i]),
                 .rd_addr_i  (rd_word_addr),
                 .rd_data_o  (vrf_rd_data[i]),
                 .rd_valid_o (vrf_rd_valid[i])
               );

      // VRF 写使能控制
      assign vrf_wr_en[i] = vrf_wr_valid_o && (wr_bank_sel == i);
      
      // VRF 读使能控制 - 支持FFT多bank读取
      assign vrf_rd_en[i] = (ld_req && (rd_bank_sel == i)) ||
                           (fft_req && (fft_state == FFT_READ_DATA) && 
                            (i >= rd_bank_sel) && (i < rd_bank_sel + 4));
    end
  endgenerate

  // VRF 读数据重组 - 支持FFT多bank读取
  always_comb
  begin
    vrf_read_data = '0;
    vrf_read_valid = 1'b0;
    
    if (fft_req && (fft_state == FFT_READ_DATA))
    begin
      // FFT需要读取4个连续bank的数据
      logic all_valid = 1'b1;
      for (int i = 0; i < 4; i++)
      begin
        if ((rd_bank_sel + i) < NumBanks)
        begin
          vrf_read_data[(i*64) +: 64] = vrf_rd_data[rd_bank_sel + i];
          all_valid = all_valid && vrf_rd_valid[rd_bank_sel + i];
        end
        else
        begin
          all_valid = 1'b0;
        end
      end
      vrf_read_valid = all_valid;
    end
    else if (ld_req)
    begin
      // 普通LOAD操作，只读取单个bank
      vrf_read_data[BankWidth-1:0] = vrf_rd_data[rd_bank_sel];
      vrf_read_valid = vrf_rd_valid[rd_bank_sel];
    end
  end

  // FFT 输入数据寄存器
  always_ff @(posedge clk_i or negedge rst_ni)
  begin
    if (!rst_ni)
    begin
      fft_X0 <= '0;
      fft_X1 <= '0;
      fft_X2 <= '0;
      fft_X3 <= '0;
    end
    else if (fft_state == FFT_READ_DATA && vrf_read_valid)
    begin
      fft_X0 <= vrf_read_data[63:0];
      fft_X1 <= vrf_read_data[127:64];
      fft_X2 <= vrf_read_data[191:128];
      fft_X3 <= vrf_read_data[255:192];
    end
  end

  // FFT结果寄存器
  always_ff @(posedge clk_i or negedge rst_ni)
  begin
    if (!rst_ni)
    begin
      fft_result_Y0 <= '0;
      fft_result_Y1 <= '0;
      fft_result_Y2 <= '0;
      fft_result_Y3 <= '0;
    end
    else if (fft_valid)
    begin
      fft_result_Y0 <= Y0;
      fft_result_Y1 <= Y1;
      fft_result_Y2 <= Y2;
      fft_result_Y3 <= Y3;
    end
  end

  // FFT状态机 - 处理3级流水线延迟
  always_ff @(posedge clk_i or negedge rst_ni)
  begin
    if (!rst_ni)
    begin
      fft_state <= FFT_IDLE;
      fft_counter <= '0;
    end
    else
    begin
      fft_state <= fft_next_state;
      
      // FFT计数器管理
      if (fft_state == FFT_COMPUTING)
        fft_counter <= fft_counter + 1;
      else if (fft_state == FFT_IDLE)
        fft_counter <= '0;
    end
  end

  always_comb
  begin
    fft_next_state = fft_state;
    
    case (fft_state)
      FFT_IDLE:
      begin
        if (fft_req)
        begin
          fft_next_state = FFT_READ_DATA;
        end
      end
      
      FFT_READ_DATA:
      begin
        if (vrf_read_valid)
        begin
          fft_next_state = FFT_COMPUTING;
        end
      end
      
      FFT_COMPUTING:
      begin
        // 等待1个时钟周期让数据稳定后启动FFT
        if (fft_counter >= 1)
        begin
          fft_next_state = FFT_WAIT_RESULT;
        end
      end
      
      FFT_WAIT_RESULT:
      begin
        if (fft_valid)  // FFT模块输出有效
        begin
          fft_next_state = FFT_DONE;
        end
      end
      
      FFT_DONE:
      begin
        if (resp_ready_i && resp_valid_o)  // 响应被接受
        begin
          fft_next_state = FFT_IDLE;
        end
      end
      
      default:
      begin
        fft_next_state = FFT_IDLE;
      end
    endcase
  end

  // FFT 控制信号
  assign fft_start = (fft_state == FFT_COMPUTING) && (fft_counter == 1);
  assign fft_busy = (fft_state == FFT_COMPUTING) || (fft_state == FFT_WAIT_RESULT);
  assign fft_ready = (fft_state == FFT_IDLE);

  // sequencer握手
  always_comb
  begin
    case (current_op_type)
      OP_FFT:
        seq_ready = (fft_state == FFT_IDLE) || (fft_state == FFT_DONE);
      default:
        seq_ready = 1'b1;
    endcase
  end

  // FFT 模块实例化
  radix4_butterfly_64bit u_fft (
                           .X0           (fft_X0),
                           .X1           (fft_X1),
                           .X2           (fft_X2),
                           .X3           (fft_X3),
                           .TF1          (seq_instr.tf1),
                           .TF2          (seq_instr.tf2),
                           .TF3          (seq_instr.tf3),
                           .clk          (clk_i),
                           .nrst         (rst_ni),
                           .enable       (fft_start),
                           .fft_available(fft_valid),
                           .Y0           (Y0),
                           .Y1           (Y1),
                           .Y2           (Y2),
                           .Y3           (Y3)
                         );

  // 响应和写回控制
  logic [MaxVLenPerLane-1:0] resp_data;
  logic resp_valid_next, vrf_wr_valid_next;
  global_vaddr_t vrf_wr_addr_next;
  lane_strb_t vrf_wr_be_next;
  logic [BankWidth-1:0] vrf_wr_data_next;

  always_comb
  begin
    // 默认值
    resp_valid_next = 1'b0;
    vrf_wr_valid_next = 1'b0;
    vrf_wr_addr_next = '0;
    vrf_wr_be_next = '0;
    vrf_wr_data_next = '0;
    resp_data = '0;

    case (current_op_type)
      OP_LOAD:
      begin
        if (vrf_read_valid && seq_ready)
        begin
          resp_valid_next = 1'b1;
          resp_data = vrf_read_data;
        end
      end

      OP_STORE:
      begin
        if (seq_valid && seq_ready)
        begin
          resp_valid_next = 1'b1;
          vrf_wr_valid_next = 1'b1;
          vrf_wr_addr_next = seq_instr.vaddr;
          vrf_wr_be_next = seq_instr.be;
          vrf_wr_data_next = seq_instr.data[BankWidth-1:0];
          resp_data = seq_instr.data;
        end
      end

      OP_FFT:
      begin
        if (fft_state == FFT_DONE)
        begin
          resp_valid_next = 1'b1;
          vrf_wr_valid_next = 1'b1;
          vrf_wr_addr_next = seq_instr.vaddr;
          vrf_wr_be_next = {BankWidth/8{1'b1}};
          vrf_wr_data_next = fft_result_Y0;  // 使用锁存的FFT结果

          resp_data[63:0]    = fft_result_Y0;
          resp_data[127:64]  = fft_result_Y1;
          resp_data[191:128] = fft_result_Y2;
          resp_data[255:192] = fft_result_Y3;
        end
      end

      default:
      begin
        // NOP 或其他情况
        if (seq_valid && seq_ready)
        begin
          resp_valid_next = 1'b1;
          resp_data = '0;
        end
      end
    endcase
  end

  // 响应和写回寄存器
  always_ff @(posedge clk_i or negedge rst_ni)
  begin
    if (!rst_ni)
    begin
      resp_valid_o   <= 1'b0;
      vrf_wr_valid_o <= 1'b0;
      vrf_wr_addr_o  <= '0;
      vrf_wr_be_o    <= '0;
      vrf_wr_data_o  <= '0;
    end
    else
    begin
      if (resp_ready_i || !resp_valid_o)
      begin
        resp_valid_o   <= resp_valid_next;
        vrf_wr_valid_o <= vrf_wr_valid_next;
        vrf_wr_addr_o  <= vrf_wr_addr_next;
        vrf_wr_be_o    <= vrf_wr_be_next;
        vrf_wr_data_o  <= vrf_wr_data_next;
      end
    end
  end

  // 响应输出赋值
  assign resp_o.vaddr = seq_instr.vaddr;
  assign resp_o.data  = resp_data;
  assign resp_o.valid = resp_valid_o;
  assign resp_o.ready = resp_ready_i;

endmodule

// VRF Bank 子模块
module vrf_bank #(
    parameter Depth     = 64,
    parameter DataWidth = 64
  )(
    input  logic               clk_i,
    input  logic               rst_ni,
    input  logic               wr_en_i,
    input  logic [5:0]         wr_addr_i,
    input  logic [DataWidth-1:0] wr_data_i,
    input  logic [7:0]         wr_be_i,
    input  logic               rd_en_i,
    input  logic [5:0]         rd_addr_i,
    output logic [DataWidth-1:0] rd_data_o,
    output logic              rd_valid_o
  );

  logic [DataWidth-1:0] mem [0:Depth-1];

  always_ff @(posedge clk_i)
  begin
    if (wr_en_i)
    begin
      for (int b = 0; b < 8; b++)
      begin
        if (wr_be_i[b])
        begin
          mem[wr_addr_i][b*8+:8] <= wr_data_i[b*8+:8];
        end
      end
    end
  end

  always_ff @(posedge clk_i)
  begin
    rd_valid_o <= rd_en_i;
    if (rd_en_i)
    begin
      rd_data_o <= mem[rd_addr_i];
    end
  end
endmodule

// Sequencer模块实现 
module lane_sequencer_enhanced #(
    parameter int unsigned OpQueueDepth  = 8,
    parameter int unsigned NrLanes       = 4,
    parameter int unsigned VLEN          = 4096,
    parameter int unsigned BytesPerLane  = 4096,
    parameter int unsigned MaxVLenPerLane= VLEN/NrLanes,
    parameter int unsigned NumBanks      = 8,
    parameter int unsigned BankWidth     = 64,
    parameter int unsigned WordsPerBank  = 64,

    parameter type global_vaddr_t = logic[11:0],
    parameter type lane_strb_t    = logic[7:0],

    parameter type pe_req_t = struct packed {
      global_vaddr_t vaddr;
      logic [31:0]   op_code;
      logic [63:0]   tf1;
      logic [63:0]   tf2;
      logic [63:0]   tf3;
      logic          wr_en;
      logic [MaxVLenPerLane-1:0] data;
      lane_strb_t    be;
    }
  )(
    input  logic          clk_i,
    input  logic          rst_ni,

    // 指令输入接口
    input  logic          instr_valid_i,
    input  pe_req_t       instr_i,
    output logic          instr_ready_o,

    // 解码后的指令输出接口
    output logic          decoded_valid_o,
    output pe_req_t       decoded_instr_o,
    input  logic          decoded_ready_i,

    // 响应接口
    input  logic          resp_valid_i,
    output logic          resp_ready_o
  );

  // 状态机状态定义
  typedef enum logic [1:0] {
            IDLE,        
            DECODE,      
            WAIT_RESP    
          } seq_state_t;

  seq_state_t state, next_state;
  pe_req_t instr_reg;

  // 状态和指令寄存器更新
  always_ff @(posedge clk_i or negedge rst_ni)
  begin
    if (!rst_ni)
    begin
      state     <= IDLE;
      instr_reg <= '0;
    end
    else
    begin
      state <= next_state;

      if (instr_valid_i && instr_ready_o)
      begin
        instr_reg <= instr_i;
      end
    end
  end

  // 状态转移逻辑
  always_comb
  begin
    next_state = state;

    case (state)
      IDLE:
      begin
        if (instr_valid_i)
        begin
          next_state = DECODE;
        end
      end

      DECODE:
      begin
        if (decoded_ready_i)
        begin
          next_state = WAIT_RESP;
        end
      end

      WAIT_RESP:
      begin
        if (resp_valid_i && resp_ready_o)
        begin
          next_state = IDLE;
        end
      end

      default:
      begin
        next_state = IDLE;
      end
    endcase
  end

  // 输出信号生成
  assign instr_ready_o   = (state == IDLE);
  assign decoded_valid_o = (state == DECODE) || (state == WAIT_RESP);
  assign resp_ready_o    = (state == WAIT_RESP);
  assign decoded_instr_o = instr_reg;

endmodule