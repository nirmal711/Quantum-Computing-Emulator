
`include "defines.vh"
//---------------------------------------------------------------------------
// DUT 
//---------------------------------------------------------------------------
module MyDesign(
//---------------------------------------------------------------------------
//System signals
  input wire reset_n                      ,  
  input wire clk                          ,

//---------------------------------------------------------------------------
//Control signals
  input wire dut_valid                    , 
  output wire dut_ready                   ,

//---------------------------------------------------------------------------
//q_state_input SRAM interface
  output wire                                               q_state_input_sram_write_enable  ,
  output wire [`Q_STATE_INPUT_SRAM_ADDRESS_UPPER_BOUND-1:0] q_state_input_sram_write_address ,
  output wire [`Q_STATE_INPUT_SRAM_DATA_UPPER_BOUND-1:0]    q_state_input_sram_write_data    ,
  output wire [`Q_STATE_INPUT_SRAM_ADDRESS_UPPER_BOUND-1:0] q_state_input_sram_read_address  , 
  input  wire [`Q_STATE_INPUT_SRAM_DATA_UPPER_BOUND-1:0]    q_state_input_sram_read_data     ,

//---------------------------------------------------------------------------
//q_state_output SRAM interface
  output wire                                                q_state_output_sram_write_enable  ,
  output wire [`Q_STATE_OUTPUT_SRAM_ADDRESS_UPPER_BOUND-1:0] q_state_output_sram_write_address ,
  output wire [`Q_STATE_OUTPUT_SRAM_DATA_UPPER_BOUND-1:0]    q_state_output_sram_write_data    ,
  output wire [`Q_STATE_OUTPUT_SRAM_ADDRESS_UPPER_BOUND-1:0] q_state_output_sram_read_address  , 
  input  wire [`Q_STATE_OUTPUT_SRAM_DATA_UPPER_BOUND-1:0]    q_state_output_sram_read_data     ,

//---------------------------------------------------------------------------
//scratchpad SRAM interface                                                       
  output wire                                                scratchpad_sram_write_enable        ,
  output wire [`SCRATCHPAD_SRAM_ADDRESS_UPPER_BOUND-1:0]     scratchpad_sram_write_address       ,
  output wire [`SCRATCHPAD_SRAM_DATA_UPPER_BOUND-1:0]        scratchpad_sram_write_data          ,
  output wire [`SCRATCHPAD_SRAM_ADDRESS_UPPER_BOUND-1:0]     scratchpad_sram_read_address        , 
  input  wire [`SCRATCHPAD_SRAM_DATA_UPPER_BOUND-1:0]        scratchpad_sram_read_data           ,

//---------------------------------------------------------------------------
//q_gates SRAM interface                                                       
  output wire                                                q_gates_sram_write_enable           ,
  output wire [`Q_GATES_SRAM_ADDRESS_UPPER_BOUND-1:0]        q_gates_sram_write_address          ,
  output wire [`Q_GATES_SRAM_DATA_UPPER_BOUND-1:0]           q_gates_sram_write_data             ,
  output wire [`Q_GATES_SRAM_ADDRESS_UPPER_BOUND-1:0]        q_gates_sram_read_address           ,  
  input  wire [`Q_GATES_SRAM_DATA_UPPER_BOUND-1:0]           q_gates_sram_read_data              
);

localparam state_0   =  4'b0000; // state 0
localparam state_1   =  4'b0001; // state 1
localparam state_2   =  4'b0010; // state 2
localparam state_3   =  4'b0011; // state 3
localparam state_4   =  4'b0100; // state 4
localparam state_5   =  4'b0101; // state 5
localparam state_6   =  4'b0110; // state 6
localparam state_7   =  4'b0111; // state 7
localparam state_8   =  4'b1000; // state 8
localparam state_9   =  4'b1001; // state 9
localparam state_10  =  4'b1010; // state 10
localparam state_11  =  4'b1011; // state 11
localparam state_12  =  4'b1100; // state 12
localparam state_13  =  4'b1101; // state 13
localparam state_14  =  4'b1110; // state 14
localparam state_15  =  4'b1111; // state 15
localparam inst_sig_width = 52;
localparam inst_exp_width = 11;
localparam inst_ieee_compliance_adder = 1;
localparam inst_ieee_compliance_mac = 0;
localparam en_ubr_flag = 0;

reg [inst_sig_width + inst_exp_width : 0] inst_a;
reg [inst_sig_width + inst_exp_width : 0] inst_b; 
reg [inst_sig_width + inst_exp_width : 0] z_inst_mult;
reg [inst_sig_width + inst_exp_width : 0] z_inst_adder;
wire [7:0] status_inst_mult, status_inst_adder;
reg [3:0] current_state, next_state;
reg set_dut_ready, get_array_size, save_array_size, input_sram_sel;
reg q_state_output_sram_write_enable_r, scratchpad_sram_write_enable_r;
reg [12:0] q_gates_sram_read_address_r, scratchpad_sram_read_address_r, scratchpad_sram_write_address_r;
reg [4:0] q_state_input_sram_read_address_r, q_state_output_sram_write_address_r; 
reg [127:0] scratchpad_sram_write_data_r, q_state_output_sram_write_data_r;
reg [4:0] m, m_counter, q_state_input_size, row_counter;
reg [7:0] q_gates_size, counter;
reg [63:0] real_adder, img_adder;
reg [1:0] m_counter_sel, row_counter_sel, counter_sel, sram_write_enable_sel, input_read_addr_sel, q_gates_read_addr_sel, scratchpad_read_addr_sel, output_write_addr_sel, scratchpad_write_addr_sel;
reg [1:0] real_adder_sel, img_adder_sel;
reg [2:0] compute_mult_adder;
reg input_sram_flag, last_element, last_row, not_last_matrix, last_matrix,  last_matrix_prev;

assign dut_ready = set_dut_ready;

always@(posedge clk or negedge reset_n) begin
  if(!reset_n)
    current_state <= 1'b0;
  else
    current_state <= next_state;
end 

always@(*) begin
  case (current_state)
  //Initial State 
  state_0:begin
    if(dut_valid) begin
      set_dut_ready = 1'b0;
      get_array_size = 1'b0;
      save_array_size = 1'b0;
      sram_write_enable_sel = 2'b00;
      input_read_addr_sel = 2'b00;
      q_gates_read_addr_sel = 2'b00;
      scratchpad_read_addr_sel = 2'b00;
      output_write_addr_sel = 2'b00;
      scratchpad_write_addr_sel = 2'b00;
      row_counter_sel = 2'b00;
      counter_sel = 2'b00;
      m_counter_sel = 2'b00;
      input_sram_sel = 1'b0;
      real_adder_sel = 2'b00;
      img_adder_sel = 2'b00;
      compute_mult_adder = 3'b000;
      next_state = state_1;
    end
    else begin
      set_dut_ready = 1'b1;
      get_array_size = 1'b0;
      save_array_size = 1'b0;
      sram_write_enable_sel = 2'b00;
      input_read_addr_sel = 2'b00;
      q_gates_read_addr_sel = 2'b00;
      scratchpad_read_addr_sel = 2'b00;
      output_write_addr_sel = 2'b00;
      scratchpad_write_addr_sel = 2'b00;
      row_counter_sel = 2'b00;
      counter_sel = 2'b00;
      m_counter_sel = 2'b00;
      input_sram_sel = 1'b0;
      real_adder_sel = 2'b00;
      img_adder_sel = 2'b00;
      compute_mult_adder = 3'b000;
      next_state = state_0;
    end
  end
  //Read size of input matrix & Select 1st element data of Q State and Q Gate SRAM 
  state_1:begin
      set_dut_ready = 1'b0;
      get_array_size = 1'b1;
      save_array_size = 1'b0;
      sram_write_enable_sel = 2'b00;
      input_read_addr_sel = 2'b01;
      q_gates_read_addr_sel = 2'b00;
      scratchpad_read_addr_sel = 2'b00;
      output_write_addr_sel = 2'b00;
      scratchpad_write_addr_sel = 2'b00;
      row_counter_sel = 2'b00;
      counter_sel = 2'b00;
      m_counter_sel = 2'b00;
      input_sram_sel = 1'b0;
      real_adder_sel = 2'b00;
      img_adder_sel = 2'b00;
      compute_mult_adder = 3'b000;
      next_state = state_2;
    end
  //Wait State for Reading Data
  state_2:begin
    set_dut_ready = 1'b0;
    get_array_size = 1'b0;
    save_array_size = 1'b1;
    sram_write_enable_sel = 2'b00;
    input_read_addr_sel = 2'b10;
    q_gates_read_addr_sel = 2'b10;
    scratchpad_read_addr_sel = 2'b10;
    output_write_addr_sel = 2'b10;
    scratchpad_write_addr_sel = 2'b10;
    row_counter_sel = 2'b10;
    counter_sel = 2'b10;
    m_counter_sel = 2'b10;
    input_sram_sel = ~input_sram_flag;
    real_adder_sel = 2'b10;
    img_adder_sel = 2'b10;
    compute_mult_adder = 3'b000;
    next_state = state_3;
  end
  //Multiply the real element of Q Gate and real element of Q State / Scratchpad 
  state_3:begin
    set_dut_ready = 1'b0;
    get_array_size = 1'b0;
    save_array_size = 1'b1;
    sram_write_enable_sel = 2'b00;
    input_read_addr_sel = 2'b10;
    q_gates_read_addr_sel = 2'b10;
    scratchpad_read_addr_sel = 2'b10;
    output_write_addr_sel = 2'b10;
    scratchpad_write_addr_sel = 2'b10;
    row_counter_sel = 2'b01;
    counter_sel = 2'b01;
    m_counter_sel = 2'b10;
    input_sram_sel = ~input_sram_flag;
    real_adder_sel = 2'b10;
    img_adder_sel = 2'b10;
    compute_mult_adder = 3'b001;
    next_state = state_4;
  end
  //Add the result of the multiplication to the fp Adder
  state_4:begin
    set_dut_ready = 1'b0;
    get_array_size = 1'b0;
    save_array_size = 1'b1;
    sram_write_enable_sel = 2'b00;
    input_read_addr_sel = 2'b10;
    q_gates_read_addr_sel = 2'b10;
    scratchpad_read_addr_sel = 2'b10;
    output_write_addr_sel = 2'b10;
    scratchpad_write_addr_sel = 2'b10;
    row_counter_sel = 2'b10;
    counter_sel = 2'b10;
    m_counter_sel = 2'b10;
    input_sram_sel = ~input_sram_flag;
    real_adder_sel = 2'b10;
    img_adder_sel = 2'b10;
    compute_mult_adder = 3'b101;
    next_state = state_5;
  end
  //Multiply the imaginary element of Q Gate and imaginary element of Q State / Scratchpad element
  //Store the result of fp adder result in real_adder
  state_5:begin
    set_dut_ready = 1'b0;
    get_array_size = 1'b0;
    save_array_size = 1'b1;
    sram_write_enable_sel = 2'b00;
    input_read_addr_sel = 2'b10;
    q_gates_read_addr_sel = 2'b10;
    scratchpad_read_addr_sel = 2'b10;
    output_write_addr_sel = 2'b10;
    scratchpad_write_addr_sel = 2'b10;
    row_counter_sel = 2'b10;
    counter_sel = 2'b10;
    m_counter_sel = 2'b10;
    input_sram_sel = ~input_sram_flag;
    real_adder_sel = 2'b01;
    img_adder_sel = 2'b10;
    compute_mult_adder = 3'b010;
    next_state = state_6;
  end
  //Add the result of the multiplication to the fp Adder
  state_6:begin
    set_dut_ready = 1'b0;
    get_array_size = 1'b0;
    save_array_size = 1'b1;
    sram_write_enable_sel = 2'b00;
    input_read_addr_sel = 2'b10;
    q_gates_read_addr_sel = 2'b10;
    scratchpad_read_addr_sel = 2'b10;
    output_write_addr_sel = 2'b10;
    scratchpad_write_addr_sel = 2'b10;
    row_counter_sel = 2'b10;
    counter_sel = 2'b10;
    m_counter_sel = 2'b10;
    input_sram_sel = ~input_sram_flag;
    real_adder_sel = 2'b10;
    img_adder_sel = 2'b10;
    compute_mult_adder = 3'b110;
    next_state = state_7;
  end
  //Multiply the real element of Q Gate and imaginary element of Q State / Scratchpad element
  //Store the result of fp adder result in real_adder
  state_7:begin
    set_dut_ready = 1'b0;
    get_array_size = 1'b0;
    save_array_size = 1'b1;
    sram_write_enable_sel = 2'b00;
    input_read_addr_sel = 2'b10;
    q_gates_read_addr_sel = 2'b10;
    scratchpad_read_addr_sel = 2'b10;
    output_write_addr_sel = 2'b10;
    scratchpad_write_addr_sel = 2'b10;
    row_counter_sel = 2'b10;
    counter_sel = 2'b10;
    m_counter_sel = 2'b10;
    input_sram_sel = ~input_sram_flag;
    real_adder_sel = 2'b01;
    img_adder_sel = 2'b10;
    compute_mult_adder = 3'b011;
    next_state = state_8;
  end
  //Add the result multiplication to the fp Adder
  state_8:begin
    set_dut_ready = 1'b0;
    get_array_size = 1'b0;
    save_array_size = 1'b1;
    sram_write_enable_sel = 2'b00;
    input_read_addr_sel = 2'b10;
    q_gates_read_addr_sel = 2'b10;
    scratchpad_read_addr_sel = 2'b10;
    output_write_addr_sel = 2'b10;
    scratchpad_write_addr_sel = 2'b10;
    row_counter_sel = 2'b10;
    counter_sel = 2'b10;
    m_counter_sel = 2'b10;
    input_sram_sel = ~input_sram_flag;
    real_adder_sel = 2'b10;
    img_adder_sel = 2'b10;
    compute_mult_adder = 3'b111;
    next_state = state_9;
  end
  //Multiply the imaginary element of Q Gate and real element of Q State / Scratchpad element
  //Store the result of fp adder result in img_adder
  state_9:begin
    set_dut_ready = 1'b0;
    get_array_size = 1'b0;
    save_array_size = 1'b1;
    sram_write_enable_sel = 2'b00;
    input_read_addr_sel = 2'b10;
    q_gates_read_addr_sel = 2'b10;
    scratchpad_read_addr_sel = 2'b10;
    output_write_addr_sel = 2'b10;
    scratchpad_write_addr_sel = 2'b10;
    row_counter_sel = 2'b10;
    counter_sel = 2'b10;
    m_counter_sel = 2'b10;
    input_sram_sel = ~input_sram_flag;
    real_adder_sel = 2'b10;
    img_adder_sel = 2'b01;
    compute_mult_adder = 3'b100;
    next_state = state_10;
  end
  //Add the result multiplication to the fp Adder
  state_10:begin
    set_dut_ready = 1'b0;
    get_array_size = 1'b0;
    save_array_size = 1'b1;
    sram_write_enable_sel = 2'b00;
    input_read_addr_sel = 2'b10;
    q_gates_read_addr_sel = 2'b10;
    scratchpad_read_addr_sel = 2'b10;
    output_write_addr_sel = 2'b10;
    scratchpad_write_addr_sel = 2'b10;
    row_counter_sel = 2'b10;
    counter_sel = 2'b10;
    m_counter_sel = 2'b10;
    input_sram_sel = ~input_sram_flag;
    real_adder_sel = 2'b10;
    img_adder_sel = 2'b10;
    compute_mult_adder = 3'b111;
    next_state = state_11;
  end
  //Store the result of fp adder result in img_adder
  //Check end of row
  state_11:begin
    set_dut_ready = 1'b0;
    get_array_size = 1'b0;
    save_array_size = 1'b1;
    sram_write_enable_sel = 2'b00;
    output_write_addr_sel = 2'b10;
    scratchpad_write_addr_sel = 2'b10;
    row_counter_sel = 2'b10;
    counter_sel = 2'b10;
    m_counter_sel = 2'b10;
    input_sram_sel = ~input_sram_flag;
    real_adder_sel = 2'b10;
    img_adder_sel = 2'b01;
    compute_mult_adder = 3'b000;
    if (last_row) begin
      input_read_addr_sel = 2'b10;
      q_gates_read_addr_sel = 2'b10;
      scratchpad_read_addr_sel = 2'b10;
      next_state = state_12;
    end
    else begin
      input_read_addr_sel = 2'b01;
      q_gates_read_addr_sel = 2'b01;
      scratchpad_read_addr_sel = (input_sram_flag) ? 2'b10 : 2'b01;
      next_state = state_2;
    end
  end
  //Save the result
  state_12:begin
    set_dut_ready = 1'b0;
    get_array_size = 1'b0;
    save_array_size = 1'b1;
    sram_write_enable_sel = (last_matrix_prev) ? 2'b01 : 2'b10;
    input_read_addr_sel = 2'b10;
    q_gates_read_addr_sel = 2'b10;
    scratchpad_read_addr_sel = 2'b10;
    output_write_addr_sel = 2'b10;
    scratchpad_write_addr_sel = 2'b10;
    row_counter_sel = 2'b00;
    counter_sel = 2'b10;
    m_counter_sel = 2'b10;
    input_sram_sel = ~input_sram_flag;
    real_adder_sel = 2'b10;
    img_adder_sel = 2'b10;
    compute_mult_adder = 2'b00;
    next_state = state_13;
  end
  //Wait State for Saving Data
    state_13:begin
    set_dut_ready = 1'b0;
    get_array_size = 1'b0;
    save_array_size = 1'b1;
    sram_write_enable_sel = 2'b00;
    input_read_addr_sel = 2'b10;
    q_gates_read_addr_sel = 2'b10;
    scratchpad_read_addr_sel = 2'b10;
    output_write_addr_sel = (last_matrix_prev) ? 2'b01 : 2'b10;
    scratchpad_write_addr_sel = 2'b10;
    row_counter_sel = 2'b10;
    counter_sel = 2'b10;
    m_counter_sel = (last_element) ? 2'b01 : 2'b10;
    input_sram_sel = ~input_sram_flag;
    real_adder_sel = 2'b10;
    img_adder_sel = 2'b10;
    compute_mult_adder = 2'b00;
    next_state = state_14;
  end
  //Check if end of matrix
  state_14:begin
    set_dut_ready = 1'b0;
    get_array_size = 1'b0;
    save_array_size = 1'b1;
    sram_write_enable_sel = 2'b00;
    q_gates_read_addr_sel = 2'b01;
    output_write_addr_sel = 2'b10;
    scratchpad_write_addr_sel = 2'b01;
    row_counter_sel = 2'b00;
    m_counter_sel = 2'b10;
    input_sram_sel = ~input_sram_flag;
    real_adder_sel = 2'b00;
    img_adder_sel = 2'b00;
    compute_mult_adder = 2'b00;
    if (last_element && last_matrix) begin
      input_read_addr_sel = 2'b10;
      scratchpad_read_addr_sel = (input_sram_flag) ? 2'b10 : 2'b10;
      counter_sel = 2'b00;
      next_state = state_15;
    end
    else if (last_element && not_last_matrix) begin
      input_read_addr_sel = 2'b10;
      scratchpad_read_addr_sel = (input_sram_flag) ? 2'b10 : 2'b11;
      counter_sel = 2'b00;
      next_state = state_2;
    end
    else begin 
      input_read_addr_sel = 2'b11;
      scratchpad_read_addr_sel = (input_sram_flag) ? 2'b10 : 2'b11;
      counter_sel = 2'b10;
      next_state = state_2;
    end
  end
  //Complete State
  state_15:begin
    set_dut_ready = 1'b1;
    get_array_size = 1'b0;
    save_array_size = 1'b0;
    sram_write_enable_sel = 2'b00;
    input_read_addr_sel = 2'b00;
    q_gates_read_addr_sel = 2'b00;
    scratchpad_read_addr_sel = 2'b00;
    output_write_addr_sel = 2'b00;
    scratchpad_write_addr_sel = 2'b00;
    row_counter_sel = 2'b00;
    counter_sel = 2'b00;
    m_counter_sel = 2'b00;
    input_sram_sel = 1'b0;
    real_adder_sel = 2'b00;
    img_adder_sel = 2'b00;
    compute_mult_adder = 2'b00;
    next_state = state_0;
  end
  default:begin
    set_dut_ready = 1'b1;
    get_array_size = 1'b0;
    save_array_size = 1'b0;
    sram_write_enable_sel = 2'b00;
    input_read_addr_sel = 2'b00;
    q_gates_read_addr_sel = 2'b00;
    scratchpad_read_addr_sel = 2'b00;
    output_write_addr_sel = 2'b00;
    scratchpad_write_addr_sel = 2'b00;
    row_counter_sel = 2'b00;
    counter_sel = 2'b00;
    m_counter_sel = 2'b00;
    input_sram_sel = 1'b0;
    real_adder_sel = 2'b00;
    img_adder_sel = 2'b00;
    compute_mult_adder = 2'b00;
    next_state = state_0;
  end
  endcase
end

// Find the number of array elements 
always @(posedge clk) begin : proc_array_size
  if (!reset_n) begin
    m <= 1'b0;
    q_state_input_size <= 1'b0;
    q_gates_size <= 1'b0;
  end else begin
    if (get_array_size) begin
      m <= q_state_input_sram_read_data[63:0];
      q_state_input_size <= 1 << q_state_input_sram_read_data[127:64];
      q_gates_size <= (1 << q_state_input_sram_read_data[127:64]) * (1 << q_state_input_sram_read_data[127:64]);  
    end 
    else if (!save_array_size) begin 
      m <= 1'b0;
      q_state_input_size <= 1'b0;
      q_gates_size <= 1'b0; 
    end 
  end
end

//Flag logic
always @(posedge clk) begin : proc_flag
  if (!reset_n) begin
    input_sram_flag <= 1'b0;
    not_last_matrix <= 1'b0;
    last_matrix <= 1'b0;
    last_matrix_prev <= 1'b0;
    last_element <= 1'b0;
    last_row <= 1'b0;
  end else begin
    input_sram_flag <= (!m_counter);
    not_last_matrix <= (m_counter != m);
    last_matrix <= (m_counter == m);
    last_matrix_prev <= (m_counter + 1'b1 == m);
    last_element <= (counter == q_gates_size);
    last_row <= (row_counter == q_state_input_size);
  end
end

// SRAM write enable logic
always @(posedge clk) begin : proc_sram_input_write_enable_r
  if(!reset_n) begin
    q_state_output_sram_write_enable_r <= 1'b0; 
    scratchpad_sram_write_enable_r <= 1'b0;
  end 
  else begin
    case (sram_write_enable_sel)
      2'b01: begin
        q_state_output_sram_write_enable_r <= 1'b1; 
        scratchpad_sram_write_enable_r <= 1'b0;
      end
      2'b10: begin
        q_state_output_sram_write_enable_r <= 1'b0; 
        scratchpad_sram_write_enable_r <= 1'b1;
      end
      default: begin
        q_state_output_sram_write_enable_r <= 1'b0; 
        scratchpad_sram_write_enable_r <= 1'b0;
      end
    endcase
  end
end

assign q_state_input_sram_write_enable = 1'b0;
assign q_gates_sram_write_enable = 1'b0;
assign q_state_output_sram_write_enable = q_state_output_sram_write_enable_r;
assign scratchpad_sram_write_enable = scratchpad_sram_write_enable_r;

//q_state Input SRAM read address generator
always @(posedge clk) begin : proc_q_state_input_sram_read_address_r
  if (!reset_n) begin
    q_state_input_sram_read_address_r <= 1'b0;
  end
  else begin
    case (input_read_addr_sel) 
      2'b00: q_state_input_sram_read_address_r <= 1'b0;
      2'b01: q_state_input_sram_read_address_r <= q_state_input_sram_read_address_r + 1'b1;
      2'b10: q_state_input_sram_read_address_r <= q_state_input_sram_read_address_r;
      2'b11: q_state_input_sram_read_address_r <= 1'b1;
    endcase
  end
end

assign q_state_input_sram_read_address = q_state_input_sram_read_address_r;

//q_gate Input SRAM read address generator
always @(posedge clk) begin : proc_q_gates_sram_read_address_r
  if (!reset_n) begin
    q_gates_sram_read_address_r <= 1'b0;
  end
  else begin
    case (q_gates_read_addr_sel) 
      2'b01: q_gates_sram_read_address_r <= q_gates_sram_read_address_r + 1'b1;
      2'b10: q_gates_sram_read_address_r <= q_gates_sram_read_address_r;
      default: q_gates_sram_read_address_r <= 1'b0;
    endcase
  end
end

assign q_gates_sram_read_address = q_gates_sram_read_address_r;

//Scratchpad SRAM read address generator
always @(posedge clk) begin : proc_scratchpad_sram_read_address_r
  if (!reset_n) begin
    scratchpad_sram_read_address_r <= 1'b0;
  end
  else begin
    case (scratchpad_read_addr_sel) 
      2'b00: scratchpad_sram_read_address_r <= 1'b0;
      2'b01: scratchpad_sram_read_address_r <= scratchpad_sram_read_address_r + 1'b1;
      2'b10: scratchpad_sram_read_address_r <= scratchpad_sram_read_address_r;
      2'b11: scratchpad_sram_read_address_r <= (m_counter - 1'b1) * q_state_input_size;
    endcase
  end
end

assign scratchpad_sram_read_address = scratchpad_sram_read_address_r; 

// SRAM result write address logic
always @(posedge clk) begin : proc_sram_result_write_address_r
  if(!reset_n) begin
    q_state_output_sram_write_address_r <= 1'b0;
  end else begin
    case (output_write_addr_sel) 
      2'b01: q_state_output_sram_write_address_r <= q_state_output_sram_write_address_r + 1'b1;
      2'b10: q_state_output_sram_write_address_r <= q_state_output_sram_write_address_r;
      default: q_state_output_sram_write_address_r <= 1'b0;
    endcase
  end
end

assign q_state_output_sram_write_address = q_state_output_sram_write_address_r; 

// SRAM scratch write address logic
always @(posedge clk) begin : proc_sram_scratch_write_address_r
  if(!reset_n) begin
    scratchpad_sram_write_address_r <= 1'b0;
  end else begin
    case (scratchpad_write_addr_sel) 
      2'b01: scratchpad_sram_write_address_r <= scratchpad_sram_write_address_r + 1'b1;
      2'b10: scratchpad_sram_write_address_r <= scratchpad_sram_write_address_r;
      default: scratchpad_sram_write_address_r <= 1'b0;
    endcase
  end
end

assign scratchpad_sram_write_address = scratchpad_sram_write_address_r; 

// SRAM result write data logic
always @(posedge clk) begin : proc_q_state_output_sram_write_data
  if(!reset_n)
    q_state_output_sram_write_data_r <= 1'b0;
  else 
    q_state_output_sram_write_data_r <= (sram_write_enable_sel == 2'b01) ? {real_adder, img_adder} : 1'b0;
end

assign q_state_output_sram_write_data = q_state_output_sram_write_data_r;

// SRAM scratch write data logic
always @(posedge clk) begin : proc_scratchpad_sram_write_data
  if(!reset_n)
    scratchpad_sram_write_data_r <= 1'b0;
  else 
    scratchpad_sram_write_data_r <= (sram_write_enable_sel == 2'b10) ? {real_adder, img_adder} : 1'b0;
end

assign scratchpad_sram_write_data = scratchpad_sram_write_data_r;

//Row Counter
always @(posedge clk) begin : proc__row_counter
  if(!reset_n) begin
    row_counter <= 1'b0;
  end else begin
    case (row_counter_sel)
      2'b01: row_counter <= row_counter + 1'b1;
      2'b10: row_counter <= row_counter; 
      default: row_counter <= 1'b0;
    endcase
  end
end

//Counter
always @(posedge clk) begin : proc_counter
  if(!reset_n) begin
    counter <= 1'b0;
  end else begin
    case (counter_sel)
      2'b01: counter <= counter + 1'b1;
      2'b10: counter <= counter;   
      default: counter <= 1'b0; 
    endcase
  end
end

//M Counter
always @(posedge clk) begin : proc_m_counter
  if(!reset_n) begin
    m_counter <= 1'b0;
  end else begin
    case (m_counter_sel)
      2'b01: m_counter <= m_counter + 1'b1; 
      2'b10: m_counter <= m_counter;    
      default: m_counter <= 1'b0;    
    endcase
  end
end  

//Real Adder logic
always @(posedge clk) begin
  if (!reset_n) begin
    real_adder <= 1'b0;
  end else begin
    case (real_adder_sel)
      2'b01: real_adder <= z_inst_adder;   
      2'b10: real_adder <= real_adder;     
      default: real_adder <= 1'b0;        
    endcase
  end
end

//Imaginary Adder logic
always @(posedge clk) begin : proc_img_adder
  if(!reset_n) begin
    img_adder <= 1'b0;
  end 
  else begin
    case (img_adder_sel)
      2'b01: img_adder <= z_inst_adder;  
      2'b10: img_adder <= img_adder;   
      default: img_adder <= 1'b0;      
    endcase
  end
end

//Multiplication & Adder logic 
always @(posedge clk) begin : proc_mult
  if(!reset_n) begin
    inst_a      <= 1'b0;
    inst_b      <= 1'b0;
  end else begin
    case (compute_mult_adder)
      3'b001: begin
        inst_a <= q_gates_sram_read_data[127:64];
        inst_b <= (input_sram_sel) ? scratchpad_sram_read_data[127:64] : q_state_input_sram_read_data[127:64];
      end   
      3'b010: begin
        inst_a <= q_gates_sram_read_data[63:0];
        inst_b <= (input_sram_sel) ? scratchpad_sram_read_data[63:0] : q_state_input_sram_read_data[63:0];
      end   
      3'b011: begin
        inst_a <= q_gates_sram_read_data[127:64];
        inst_b <= (input_sram_sel) ? scratchpad_sram_read_data[63:0] : q_state_input_sram_read_data[63:0];
      end  
      3'b100: begin
        inst_a <= q_gates_sram_read_data[63:0]; 
        inst_b <= (input_sram_sel) ? scratchpad_sram_read_data[127:64] : q_state_input_sram_read_data[127:64];
      end
      3'b101: begin
        inst_a <= z_inst_mult;
        inst_b <= real_adder;
      end
      3'b110: begin
        inst_a <= {~z_inst_mult[63], z_inst_mult[62:0]};
        inst_b <= real_adder;
      end
      3'b111: begin
        inst_a <= z_inst_mult;
        inst_b <= img_adder;
      end
      default: begin
        inst_a <= 1'b0;
        inst_b <= 1'b0;
      end        
    endcase
  end
end

// Instance 1 of DW_fp_mult
DW_fp_mult #(inst_sig_width, inst_exp_width, inst_ieee_compliance_mac, en_ubr_flag)
U1 ( .a(inst_a), .b(inst_b), .rnd(3'b100), .z(z_inst_mult), .status(status_inst_mult) );

// Instance 1 of DW_fp_add
DW_fp_add #(inst_sig_width, inst_exp_width, inst_ieee_compliance_adder) 
U2 ( .a(inst_a), .b(inst_b), .rnd(3'b000), .z(z_inst_adder), .status(status_inst_adder) );

endmodule