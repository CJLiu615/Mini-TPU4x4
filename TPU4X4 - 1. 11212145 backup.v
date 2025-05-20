`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/14/2024 12:23:36 AM
// Design Name: 
// Module Name: TPU4X4
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

//------------------------------IR related------------------------------//
module IR_Mem(R_data, W_data, addr, clk, IR_rd, IR_wr);
    parameter address_size = 8;
    parameter word_size = 16;
    parameter memory_size = 32;
    output [word_size-1:0] R_data;
    input [word_size-1:0] W_data;
    input [address_size-1:0] addr;
    input clk, IR_rd, IR_wr;
    reg [word_size-1:0] memory [memory_size-1:0];
    
    // Initialize memory with specific values at specific addresses
    initial begin
        memory[0] = 16'b0000000000000000;
        memory[1] = 16'b0001000000000000;
        memory[2] = 16'b0010000000000001;
        memory[3] = 16'b0011000000000000;
        memory[4] = 16'b0100000000000000;
//        memory[5] = 16'b0000000000000000;
//        memory[6] = 16'b0000000000000000;
    end
    
    always @(posedge clk) begin
        if (IR_wr) begin
            memory[addr] <= W_data;
        end
    end
    assign R_data = memory[addr];
endmodule

module IR_counter(IR_addr, IR_inc, IR_clr, clk);
    parameter word_size = 8;
    output reg [word_size-1:0] IR_addr;
    input  IR_inc, IR_clr, clk;
    
    always@(posedge clk or posedge IR_clr) begin
        if(IR_clr==1) IR_addr <= 8'b0; 
        else if (IR_inc == 1) IR_addr <= IR_addr + 1;
    end
endmodule

//------------------------------Weight related------------------------------//
module Weight_DDR3(R_data, W_data, addr, clk, rd, wr);
    parameter address_size = 4;
    parameter word_size = 8;
    parameter memory_size = 16;
    output [word_size-1:0] R_data;
    input [word_size-1:0] W_data;
    input [address_size:0] addr; // [5:0]
    input clk, rd, wr;
    reg [word_size-1:0] memory [memory_size-1:0];
    
    // Initialize memory with specific values at specific addresses
    initial begin
        memory[0] = 8'b00000001; // 1
        memory[1] = 8'b00000010; // 2
        memory[2] = 8'b00000011; // 3
        memory[3] = 8'b00000100; // 4
        memory[4] = 8'b00000101; // 5
        memory[5] = 8'b00000110; // 6
        memory[6] = 8'b00000111; // 7
        memory[7] = 8'b00001000; // 8
        memory[8] = 8'b00001001; // 9
        memory[9] = 8'b00001010; // 10
        memory[10] = 8'b00001011; // 11
        memory[11] = 8'b00001100; // 12
        memory[12] = 8'b00001101; // 13
        memory[13] = 8'b00001110; // 14
        memory[14] = 8'b00001111; // 15
        memory[15] = 8'b00010000; // 16
    end

    // Write operation (triggered on clock edge)
    always @(posedge clk) begin
        if (wr) begin
            memory[addr] <= W_data; // Write data to memory at specified address
        end
    end

    assign R_data = memory[addr];

endmodule

module Weight_interface(
    clk,
    reset,
    weight_in,
    push_time,
    push,
    pop,
    pop_complete,
    out1,
    out2,
    out3,
    out4
);

    // Parameters
    parameter STACK_SIZE = 16;
    reg [7:0] stack [STACK_SIZE-1:0]; // Stack to store weights
    reg [4:0] count;              // Counter to track inputs and stack index
    input clk;
    input reset;
    input push, pop;
    input [7:0] weight_in;            // Assuming 8-bit weights for simplicity
    input [4:0] push_time;            // Signal to push weights into the stack
    output reg pop_complete;
    output reg [7:0] out1;            // Output 1
    output reg [7:0] out2;            // Output 2
    output reg [7:0] out3;            // Output 3
    output reg [7:0] out4;            // Output 4

    reg [4:0] pushtime;
    
    always@(posedge clk) begin
        pushtime <= push_time;
    end

    // Stack push phase
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            count <= 0;
        end
        if (push == 1) begin
            stack[count] <= weight_in;
            count <= count + 1;
        end
    end

    // Pop sequence control
    reg [4:0] pop_count; // Counter to control the pop sequence
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            pop_count <= 0;
            out1 <= 8'b0;
            out2 <= 8'b0;
            out3 <= 8'b0;
            out4 <= 8'b0;
            pop_complete <= 1'b0;
        end
        else if (pop == 1) begin
            // Pop weights from the stack in the specified order
            case (pop_count)
                0: begin
                out4 <= stack[15];
                out3 <= stack[14];
                out2 <= stack[13];
                out1 <= stack[12];
                pop_complete <= 1'b0;
                end
                1: begin
                out4 <= stack[11];
                out3 <= stack[10];
                out2 <= stack[9];
                out1 <= stack[8];
                pop_complete <= 1'b0;
                end
                2: begin
                out4 <= stack[7];
                out3 <= stack[6];
                out2 <= stack[5];
                out1 <= stack[4];
                pop_complete <= 1'b0;
                end
                3: begin 
                out4 <= stack[3];
                out3 <= stack[2];
                out2 <= stack[1];
                out1 <= stack[0];
                pop_complete <= 1'b0;
                end
                4: begin
                pop_complete <= 1'b1;
                end
                default: begin
                    out1 <= 8'bx;
                    out2 <= 8'bx;
                    out3 <= 8'bx;
                    out4 <= 8'bx;
                end
            endcase
            pop_count <= pop_count + 1;
        end
    end
    
endmodule

module Weight_FIFO(clk, rst, buf_in, buf_out, wr_en, rd_en, buf_empty, buf_full, fifo_counter);

    input clk, rst, wr_en, rd_en; // clock, reset, write_enable, read_enable
    input [7:0] buf_in; // input buffer
    output [7:0] buf_out; // output buffer
    output buf_empty, buf_full; // check whether buffer is empty or full
    output [7:0] fifo_counter; // check the fifo counter state(how many values had stored in the counter)
                               // must be one more byte
    
    reg [7:0] buf_out;
    reg buf_empty, buf_full;
    reg [7:0] fifo_counter;
    reg [3:0] rd_ptr, wr_ptr;
    reg [7:0] buf_mem[63:0]; // [7:0] means int, and buffer memory [63:0] means we can store up to 64 values in the memory
    
    always @(fifo_counter) begin // check the buffer memory's storage state
        buf_empty = (fifo_counter==0);
        buf_full = (fifo_counter==64);
    end
    
    always @(posedge clk or posedge rst) begin
        if(rst)
            fifo_counter <= 0; //reset counter
        else if ( (!buf_full && wr_en) && (!buf_empty && rd_en) ) // if read and write is doing at the same time
            fifo_counter <= fifo_counter; // counter should remain the same
        else if ( !buf_full && wr_en ) // write one element into the memory
            fifo_counter <= fifo_counter + 1; // counter +1
        else if ( !buf_empty && rd_en ) // reading one element out from the memory
            fifo_counter <= fifo_counter - 1; // counter -1
        else
            fifo_counter <= fifo_counter; // counter stays
    end 
    
    always @(posedge clk or posedge rst) begin
        if(rst)
            buf_out <= 0;
        else begin
            if(rd_en && !buf_empty)
                buf_out <= buf_mem[rd_ptr]; // read one element from the buffer memory
            else
                buf_out <= buf_out;
        end
    end
    
    always @(posedge clk) begin
        if(wr_en && !buf_full)
            buf_mem[wr_ptr] <= buf_in;// write one element into the buffer memory
        else
            buf_mem[wr_ptr] <= buf_mem[wr_ptr];
    end
    
    // pointer tracking 
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            wr_ptr <= 0;
            rd_ptr <= 0;
        end
        else begin
            if(!buf_full && wr_en)// head pointer
                wr_ptr <= wr_ptr + 1;
            else
                wr_ptr <= wr_ptr;
            if(!buf_empty && rd_en)// tail pointer
                rd_ptr <= rd_ptr + 1;
            else
                rd_ptr <= rd_ptr;
        end
    end
    
endmodule

//------------------------------MMU related------------------------------//
module row_detector(
    rowsignal, weight_passtime
    );
    parameter BIT_WIDTH = 8;  // Parameter for bit width
    input [BIT_WIDTH-1:0] rowsignal; // for this PE to check what row it is located, 8-bit for expansion to 256x256 MMU
    output reg [BIT_WIDTH-1:0] weight_passtime;
    
    always @(*) begin
       weight_passtime = 8'b00000011 - rowsignal; // Adjust this to match your MMU row logic
    end    
    
endmodule

module systolic_cell(
    clk, rst, row_en, col_en, weight_pass, weight_passtime,
    weightloading, // control signal for the first row to make sure the weights are loaded successfully; 1: loading, 0: completed
    activation_input, weight_input, psum_input, 
    activation_output, sum_output, weight_output
);
    parameter BIT_WIDTH = 8;  // Parameter for bit width
    input [2*BIT_WIDTH-1:0] psum_input;  // Partial sum input
    input [BIT_WIDTH-1:0] activation_input; // Activation input
    input [BIT_WIDTH-1:0] weight_input;  // Weight input
    input [BIT_WIDTH-1:0] weight_passtime; // for PE to check how many times the weight should be passed
    input rst;  // Reset signal
    input clk;  // Clock signal
    input row_en;  // Row enable signal
    input col_en;  // Column enable signal
    input weight_pass;  // Weight pass signal
    output reg weightloading;
    output reg [BIT_WIDTH-1:0] activation_output; // Activation output
    output reg [2*BIT_WIDTH-1:0] sum_output;      // Sum output
    output reg [BIT_WIDTH-1:0] weight_output;     // Weight output

    reg [BIT_WIDTH-1:0] weight_passtime_reg;
    reg [BIT_WIDTH-1:0] weight_reg; // store the weight in the reg
    reg [2*BIT_WIDTH-1:0] mac_out;  // Register to store MAC result
    
    // Combinational logic for MAC operation and output updates
    always @(*) begin
        // Default the MAC result to just passing psum_input when row_en/col_en are not active
        mac_out = psum_input;
        weight_passtime_reg = weight_passtime;
        
        // Perform MAC operation only if row and column are enabled
        if (row_en && col_en) begin
            mac_out = (activation_input * weight_reg) + psum_input;
        end
    end

    // Sequential block for reset and final output
    always @(posedge clk or posedge rst) begin
    if (rst) begin
        activation_output <= 1'b0;
        sum_output <= 1'b0;
        weight_reg <= 1'b0;
        weight_output <= 1'b0;       
    end else if (weight_pass) begin
        if (weight_passtime_reg > 0) begin
            weightloading <= 1'b1;
            weight_output <= weight_input;
            weight_passtime_reg <= weight_passtime_reg - 1;
        end else begin
            weightloading <= 1'b0;
            weight_reg <= weight_input;
            weight_output <= weight_input;
        end
    end else begin
        if (row_en && col_en) begin
            activation_output <= activation_input;
            sum_output <= mac_out;
        end else begin
            sum_output <= psum_input;
        end
    end
end
endmodule

module MMU4x4(a1, a2, a3, a4, w1, w2, w3, w4, s_in1, s_in2, s_in3, s_in4, o1, o2, o3, o4, rst, clk, w_pass, weightload_complete, r1_en, r2_en, r3_en, r4_en, c1_en, c2_en, c3_en, c4_en, rowsignal1, rowsignal2, rowsignal3, rowsignal4);
    parameter BIT_WIDTH = 8;
    // declare inputs and outputs ports
    input [BIT_WIDTH-1:0] a1, a2, a3, a4;// activation inputs
    input [BIT_WIDTH-1:0] w1, w2, w3, w4;// weight line inputs
    input [2*BIT_WIDTH-1:0] s_in1, s_in2, s_in3, s_in4;// sum inputs
    input r1_en, r2_en, r3_en, r4_en, c1_en, c2_en, c3_en, c4_en; // row and colum input
    input [BIT_WIDTH-1:0] rowsignal1, rowsignal2, rowsignal3, rowsignal4;
    input clk, rst, w_pass;
    output wire [2*BIT_WIDTH-1:0] o1, o2, o3, o4; // activation_output
    output reg weightload_complete; // 1: complete, 0: still loading
    
    //internal wires
    // vertical conntect
    // wires for passing in sum  ([n-1]*n => 3x4=12)
    wire [2*BIT_WIDTH-1:0] s_11_21, s_21_31, s_31_41, s_12_22, s_22_32, s_32_42, s_13_23, s_23_33, s_33_43, s_14_24, s_24_34, s_34_44;
    // wires for passing weights
    wire [BIT_WIDTH-1:0]  w_11_21, w_21_31, w_31_41, w_12_22, w_22_32, w_32_42, w_13_23, w_23_33, w_33_43, w_14_24, w_24_34, w_34_44;
    
    // wires for passing activations
    // horizontal conntect
    wire [BIT_WIDTH-1:0] a_11_12, a_12_13, a_13_14, a_21_22, a_22_23, a_23_24, a_31_32, a_32_33, a_33_34, a_41_42, a_42_43, a_43_44;
    
    wire weightloading1, weightloading2, weightloading3, weightloading4; // weightloading control signal for the first row
    wire [BIT_WIDTH-1:0] weight_passtime1, weight_passtime2, weight_passtime3, weight_passtime4; // from row detector
    
    always @(*) begin
        if (weightloading2 || weightloading3 || weightloading4) begin
            weightload_complete <= 1'b0; // Not complete if any cell in row 1 is still loading
        end else begin
            weightload_complete <= 1'b1; // Complete when all cells in row 1 are done loading
            end
        end 
    
    //--------------------------------ROW1--------------------------------//
    row_detector u1(
    .rowsignal(rowsignal1), .weight_passtime(weight_passtime1)
    );
    
    //--------------------------------ROW2--------------------------------//
    row_detector u2(
    .rowsignal(rowsignal2), .weight_passtime(weight_passtime2)
    );
    
    //--------------------------------ROW3--------------------------------//
    row_detector u3(
    .rowsignal(rowsignal3), .weight_passtime(weight_passtime3)
    );
    
    //--------------------------------ROW4--------------------------------//
    row_detector u4(
    .rowsignal(rowsignal4), .weight_passtime(weight_passtime4)
    );
    
    //--------------------------------COL 1--------------------------------//
    systolic_cell pe11(
    .clk(clk), .rst(rst), .row_en(r1_en), .col_en(c1_en), .weight_pass(w_pass), 
    .weight_passtime(weight_passtime1), .weightloading(weightloading1),
    .activation_input(a1), .weight_input(w1), .psum_input(s_in1), 
    .activation_output(a_11_12), .sum_output(s_11_21), .weight_output(w_11_21)
    );
    
    systolic_cell pe21(
    .clk(clk), .rst(rst), .row_en(r2_en), .col_en(c1_en), .weight_pass(w_pass), 
    .weight_passtime(weight_passtime2), .weightloading(weightloading2),
    .activation_input(a2), .weight_input(w_11_21), .psum_input(s_11_21), 
    .activation_output(a_21_22), .sum_output(s_21_31), .weight_output(w_21_31)
    );
    
    systolic_cell pe31(
    .clk(clk), .rst(rst), .row_en(r3_en), .col_en(c1_en), .weight_pass(w_pass),
    .weight_passtime(weight_passtime3), .weightloading(weightloading3), 
    .activation_input(a3), .weight_input(w_21_31), .psum_input(s_21_31), 
    .activation_output(a_31_32), .sum_output(s_31_41), .weight_output(w_31_41)
    );
    
    systolic_cell pe41(
    .clk(clk), .rst(rst), .row_en(r4_en), .col_en(c1_en), .weight_pass(w_pass),
    .weight_passtime(weight_passtime4), .weightloading(weightloading4), 
    .activation_input(a4), .weight_input(w_31_41), .psum_input(s_31_41), 
    .activation_output(a_41_42), .sum_output(o1), .weight_output()
    );
    
    
    //--------------------------------COL 2--------------------------------//
    systolic_cell pe12(
    .clk(clk), .rst(rst), .row_en(r1_en), .col_en(c2_en), .weight_pass(w_pass),
    .weight_passtime(weight_passtime1), .weightloading(), 
    .activation_input(a_11_12), .weight_input(w2), .psum_input(s_in2), 
    .activation_output(a_12_13), .sum_output(s_12_22), .weight_output(w_12_22)
    );
    
    systolic_cell pe22(
    .clk(clk), .rst(rst), .row_en(r2_en), .col_en(c2_en), .weight_pass(w_pass),
    .weight_passtime(weight_passtime2), .weightloading(), 
    .activation_input(a_21_22), .weight_input(w_12_22), .psum_input(s_12_22), 
    .activation_output(a_22_23), .sum_output(s_22_32), .weight_output(w_22_32)
    );
    
    systolic_cell pe32(
    .clk(clk), .rst(rst), .row_en(r3_en), .col_en(c2_en), .weight_pass(w_pass),
    .weight_passtime(weight_passtime3), .weightloading(),
    .activation_input(a_31_32), .weight_input(w_22_32), .psum_input(s_22_32), 
    .activation_output(a_32_33), .sum_output(s_32_42), .weight_output(w_32_42)
    );
    
    systolic_cell pe42(
    .clk(clk), .rst(rst), .row_en(r4_en), .col_en(c2_en), .weight_pass(w_pass), 
    .weight_passtime(weight_passtime4), .weightloading(),
    .activation_input(a_41_42), .weight_input(w_32_42), .psum_input(s_32_42), 
    .activation_output(a_42_43), .sum_output(o2), .weight_output()
    );
    
    //--------------------------------COL 3--------------------------------//
    systolic_cell pe13(
    .clk(clk), .rst(rst), .row_en(r1_en), .col_en(c3_en), .weight_pass(w_pass),
    .weight_passtime(weight_passtime1), .weightloading(), 
    .activation_input(a_12_13), .weight_input(w3), .psum_input(s_in3), 
    .activation_output(a_13_14), .sum_output(s_13_23), .weight_output(w_13_23)
    );
    
    systolic_cell pe23(
    .clk(clk), .rst(rst), .row_en(r2_en), .col_en(c3_en), .weight_pass(w_pass), 
    .weight_passtime(weight_passtime2), .weightloading(),
    .activation_input(a_22_23), .weight_input(w_13_23), .psum_input(s_13_23), 
    .activation_output(a_23_24), .sum_output(s_23_33), .weight_output(w_23_33)
    );
    
    systolic_cell pe33(
    .clk(clk), .rst(rst), .row_en(r3_en), .col_en(c3_en), .weight_pass(w_pass),
    .weight_passtime(weight_passtime3), .weightloading(), 
    .activation_input(a_32_33), .weight_input(w_23_33), .psum_input(s_23_33), 
    .activation_output(a_33_34), .sum_output(s_33_43), .weight_output(w_33_43)
    );
    
    systolic_cell pe43(
    .clk(clk), .rst(rst), .row_en(r4_en), .col_en(c3_en), .weight_pass(w_pass),
    .weight_passtime(weight_passtime4), .weightloading(),
    .activation_input(a_42_43), .weight_input(w_33_43), .psum_input(s_33_43), 
    .activation_output(a_43_44), .sum_output(o3), .weight_output()
    );
    
    //--------------------------------COL 4--------------------------------//
    systolic_cell pe14(
    .clk(clk), .rst(rst), .row_en(r1_en), .col_en(c4_en), .weight_pass(w_pass),
    .weight_passtime(weight_passtime1), .weightloading(), 
    .activation_input(a_13_14), .weight_input(w4), .psum_input(s_in4), 
    .activation_output(), .sum_output(s_14_24), .weight_output(w_14_24)
    );
    
    systolic_cell pe24(
    .clk(clk), .rst(rst), .row_en(r2_en), .col_en(c4_en), .weight_pass(w_pass), 
    .weight_passtime(weight_passtime2), .weightloading(),
    .activation_input(a_23_24), .weight_input(w_14_24), .psum_input(s_14_24), 
    .activation_output(), .sum_output(s_24_34), .weight_output(w_24_34)
    );
    
    systolic_cell pe34(
    .clk(clk), .rst(rst), .row_en(r3_en), .col_en(c4_en), .weight_pass(w_pass), 
    .weight_passtime(weight_passtime3), .weightloading(),
    .activation_input(a_33_34), .weight_input(w_24_34), .psum_input(s_24_34), 
    .activation_output(), .sum_output(s_34_44), .weight_output(w_34_44)
    );
    
    systolic_cell pe44(
    .clk(clk), .rst(rst), .row_en(r4_en), .col_en(c4_en), .weight_pass(w_pass), 
    .weight_passtime(weight_passtime4), .weightloading(),
    .activation_input(a_43_44), .weight_input(w_34_44), .psum_input(s_34_44), 
    .activation_output(), .sum_output(o4), .weight_output()
    );

endmodule

//------------------------------Other Computation related------------------------------//
//------------------------------before MMU------------------------------//
module unified_buffer(
    addr, // maybe for host memory to transfer data
    clk, 
    rd, // read_enable = 1, data goes from UB to SDS
    wr, // write_enable =1, data goes form NU to UB
    counter_rst, // reset the counter i for data storage address reorient
    data_in1, data_in2, data_in3, data_in4, // data in from NU
    // data out that goes to SDS
    data_out1, data_out2, data_out3, data_out4, data_out5, data_out6, data_out7, data_out8, 
    data_out9, data_out10, data_out11, data_out12, data_out13, data_out14, data_out15, data_out16
    );
    parameter BIT_WIDTH = 8;
    parameter address_size = 4;
    parameter word_size = 8;
    parameter memory_size = 16;
    // input from normalization unit
    input [word_size-1:0] data_in1, data_in2, data_in3, data_in4;
    // output to sds
    output reg [word_size-1:0] data_out1, data_out2, data_out3, data_out4, data_out5, data_out6, data_out7, data_out8, data_out9, data_out10, data_out11, data_out12, data_out13, data_out14, data_out15, data_out16;
    // input [word_size-1:0] W_data;
    input [address_size-1:0] addr;
    input counter_rst; // rst the counter i
    input clk;
    input rd, wr; // read to sds, write from normalization unit
    reg [word_size-1:0] memory [memory_size-1:0];
    
    // Initialize memory with specific values at specific addresses
    initial begin
        memory[0] = 8'b00000001; // 1
        memory[1] = 8'b00000010; // 2
        memory[2] = 8'b00000011; // 3
        memory[3] = 8'b00000100; // 4
        memory[4] = 8'b00000101; // 5
        memory[5] = 8'b00000110; // 6
        memory[6] = 8'b00000111; // 7
        memory[7] = 8'b00001000; // 8
        memory[8] = 8'b00001001; // 9
        memory[9] = 8'b00001010; // 10
        memory[10] = 8'b00001011; // 11
        memory[11] = 8'b00001100; // 12
        memory[12] = 8'b00001101; // 13
        memory[13] = 8'b00001110; // 14
        memory[14] = 8'b00001111; // 15
        memory[15] = 8'b00010000; // 16
    end
    
    reg [BIT_WIDTH-1:0] i;
    
    // Reset counter when counter_rst switches from 0 to 1
    always @(posedge clk) begin
        if (counter_rst) begin
            i <= 0;
        end 
        else begin
            i <= i + 1;
        end
    end
    
    // Memory write and read control
    always @(posedge clk) begin
        if (wr && !rd) begin
            case (i)
                3'd0: begin
                      memory[0] <= data_in1;
                      memory[1] <= data_in2;
                      memory[2] <= data_in3;
                      memory[3] <= data_in4;
                      end
                3'd1: begin
                      memory[4] <= data_in1;
                      memory[5] <= data_in2;
                      memory[6] <= data_in3;
                      memory[7] <= data_in4;
                      end
                3'd2: begin
                      memory[8] <= data_in1;
                      memory[9] <= data_in2;
                      memory[10] <= data_in3;
                      memory[11] <= data_in4;
                      end
                3'd3: begin
                      memory[12] <= data_in1;
                      memory[13] <= data_in2;
                      memory[14] <= data_in3;
                      memory[15] <= data_in4; // Final expected write to memory[15]
                      end
                default: ; // No action, ensuring no unintended overwrites
            endcase
        end 
        if (rd && !wr) begin
            // Assign outputs based on memory values
            data_out1 <= memory[0];
            data_out2 <= memory[1];
            data_out3 <= memory[2];
            data_out4 <= memory[3];
            data_out5 <= memory[4];
            data_out6 <= memory[5];
            data_out7 <= memory[6];
            data_out8 <= memory[7];
            data_out9 <= memory[8];
            data_out10 <= memory[9];
            data_out11 <= memory[10];
            data_out12 <= memory[11];
            data_out13 <= memory[12];
            data_out14 <= memory[13];
            data_out15 <= memory[14];
            data_out16 <= memory[15];
        end
    end
endmodule

module sds4x4(
    clk,
    counter_rst, sds_work,
    a0, a1, a2, a3, b0, b1, b2, b3, c0, c1, c2, c3, d0, d1, d2, d3,
    output1, output2, output3, output4
);
    parameter BIT_WIDTH = 8;  // Parameter for bit width
    // clock
    input clk;
    // counter reset signal (from control signal, reset the integer i used in this component)
    input counter_rst;
    input sds_work;
    //8 inputs, each contains 4 bits
    input [BIT_WIDTH-1:0] a0, a1, a2, a3, b0, b1, b2, b3, c0, c1, c2, c3, d0, d1, d2, d3;
    //256 outputs for the data rearrangement
    output reg [BIT_WIDTH-1:0] output1;
    output reg [BIT_WIDTH-1:0] output2;
    output reg [BIT_WIDTH-1:0] output3;
    output reg [BIT_WIDTH-1:0] output4;
    
    reg [BIT_WIDTH-1:0] i;
    
    // Reset counter when counter_rst switches from 0 to 1
    always @(posedge clk) begin
        if (counter_rst) begin
            i <= 0;
        end 
        else if (sds_work) begin
            i <= i + 1;
        end
    end

   // Output control based on the cycle counter
    always @(posedge clk) begin
        if (sds_work) begin
            case (i)
                3'd0: begin    
                    output1 <= a0;
//                    output2 <= 8'b0;
//                    output3 <= 8'b0;
//                    output4 <= 8'b0;
                end
                3'd1: begin
                    output1 <= b0;
                    output2 <= a1;
                    output3 <= 8'b0;
                    output4 <= 8'b0;
                end
                3'd2: begin
                    output1 <= c0;
                    output2 <= b1;
                    output3 <= a2;
                    output4 <= 8'b0;
                end
                3'd3: begin
                    output1 <= d0;
                    output2 <= c1;
                    output3 <= b2;
                    output4 <= a3;
                end
                3'd4: begin
                    output1 <= 8'b0;
                    output2 <= d1;
                    output3 <= c2;
                    output4 <= b3;
                end
                3'd5: begin
                    output1 <= 8'b0;
                    output2 <= 8'b0;
                    output3 <= d2;
                    output4 <= c3;
                end
                3'd6: begin
                    output1 <= 8'b0;
                    output2 <= 8'b0;
                    output3 <= 8'b0;
                    output4 <= d3;
                end
                default: begin
                    output1 <= 8'b0;
                    output2 <= 8'b0;
                    output3 <= 8'b0;
                    output4 <= 8'b0;
                end
            endcase
        end
    end
endmodule

module Activation_FIFO(clk, rst, buf_in, buf_out, wr_en, rd_en, buf_empty, buf_full, fifo_counter);

    input clk, rst, wr_en, rd_en; // clock, reset, write_enable, read_enable
    input [7:0] buf_in; // input buffer
    output [7:0] buf_out; // output buffer
    output buf_empty, buf_full; // check whether buffer is empty or full
    output [7:0] fifo_counter; // check the fifo counter state(how many values had stored in the counter)
                               // must be one more byte
    
    reg [7:0] buf_out;
    reg buf_empty, buf_full;
    reg [7:0] fifo_counter;
    reg [3:0] rd_ptr, wr_ptr;
    reg [7:0] buf_mem[63:0]; // [7:0] means int, and buffer memory [63:0] means we can store up to 64 values in the memory
    
    always @(fifo_counter) begin // check the buffer memory's storage state
        buf_empty = (fifo_counter==0);
        buf_full = (fifo_counter==64);
    end
    
    always @(posedge clk or posedge rst) begin
        if(rst)
            fifo_counter <= 0; //reset counter
        else if ( (!buf_full && wr_en) && (!buf_empty && rd_en) ) // if read and write is doing at the same time
            fifo_counter <= fifo_counter; // counter should remain the same
        else if ( !buf_full && wr_en ) // write one element into the memory
            fifo_counter <= fifo_counter + 1; // counter +1
        else if ( !buf_empty && rd_en ) // reading one element out from the memory
            fifo_counter <= fifo_counter - 1; // counter -1
        else
            fifo_counter <= fifo_counter; // counter stays
    end 
    
    always @(posedge clk or posedge rst) begin
        if(rst)
            buf_out <= 0;
        else begin
            if(rd_en && !buf_empty)
                buf_out <= buf_mem[rd_ptr]; // read one element from the buffer memory
            else
                buf_out <= buf_out;
        end
    end
    
    always @(posedge clk) begin
        if(wr_en && !buf_full)
            buf_mem[wr_ptr] <= buf_in;// write one element into the buffer memory
        else
            buf_mem[wr_ptr] <= buf_mem[wr_ptr];
    end
    
    // pointer tracking 
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            wr_ptr <= 0;
            rd_ptr <= 0;
        end
        else begin
            if(!buf_full && wr_en)// head pointer
                wr_ptr <= wr_ptr + 1;
            else
                wr_ptr <= wr_ptr;
            if(!buf_empty && rd_en)// tail pointer
                rd_ptr <= rd_ptr + 1;
            else
                rd_ptr <= rd_ptr;
        end
    end
    
endmodule

//------------------------------After MMU------------------------------//
module Accumulator4x4(
    MMU_size,
    write_enable,
    read_enable,
    in1, in2, in3, in4,
    out1, out2, out3, out4,
    accumulator_finish_storing,
    clk, rst
);
    parameter BIT_WIDTH = 8;
    parameter word_size = 16;
    parameter memory_size = 4;

    input [BIT_WIDTH-1:0] MMU_size; // Size input
    input [2*BIT_WIDTH-1:0] in1, in2, in3, in4; // Input signals
    input clk, rst;
    input write_enable, read_enable;
    output reg [2*BIT_WIDTH-1:0] out1, out2, out3, out4; // Output signals
    output reg accumulator_finish_storing; // Signal to indicate when memory is full

    // Memory arrays for storing inputs
    reg [word_size-1:0] memory1 [memory_size-1:0];
    reg [word_size-1:0] memory2 [memory_size-1:0];
    reg [word_size-1:0] memory3 [memory_size-1:0];
    reg [word_size-1:0] memory4 [memory_size-1:0];

    // Cycle counters for controlling storage timing
    reg [7:0] cycle_count1, cycle_count2, cycle_count3, cycle_count4; 

    // Output logic: Sequentially output values from memory arrays when read_enable is high
    reg [1:0] read_index;  // Index to iterate through memory arrays for output
    
    // Memory storage logic with cycle counting for each input
    always @(posedge clk) begin
        if (rst) begin
            // Reset counters on reset
            cycle_count1 <= 0;
            cycle_count2 <= 0;
            cycle_count3 <= 0;
            cycle_count4 <= 0;
            accumulator_finish_storing <= 1'b0;
            read_index <= 0;
        end 
        if (write_enable) begin
            // Increment cycle counters
            if (cycle_count1 < MMU_size + 5) cycle_count1 <= cycle_count1 + 1;
            if (cycle_count2 < MMU_size + 6) cycle_count2 <= cycle_count2 + 1;
            if (cycle_count3 < MMU_size + 7) cycle_count3 <= cycle_count3 + 1;
            if (cycle_count4 < MMU_size + 8) cycle_count4 <= cycle_count4 + 1;

            // Store inputs into memory arrays based on cycle counts
            if (cycle_count1 >= 5 && cycle_count1 < (5 + MMU_size)) memory1[cycle_count1 - 5] <= in1;
            if (cycle_count2 >= 6 && cycle_count2 < (6 + MMU_size)) memory2[cycle_count2 - 6] <= in2;
            if (cycle_count3 >= 7 && cycle_count3 < (7 + MMU_size)) memory3[cycle_count3 - 7] <= in3;
            if (cycle_count4 >= 8 && cycle_count4 < (8 + MMU_size)) memory4[cycle_count4 - 8] <= in4;

            // Set finish signal when all cycle counts reach their limits
            if ((cycle_count1 >= MMU_size + 4) &&
                (cycle_count2 >= MMU_size + 5) &&
                (cycle_count3 >= MMU_size + 6) &&
                (cycle_count4 >= MMU_size + 7)) begin
                accumulator_finish_storing <= 1'b1;
            end
        end else begin
            // Reset finish storing flag when write_enable is de-asserted
            accumulator_finish_storing <= 1'b0;
        end
    end

    always @(posedge clk) begin
        if (read_enable) begin
            if (read_index < memory_size) begin
                out1 <= memory1[read_index];
                out2 <= memory2[read_index];
                out3 <= memory3[read_index];
                out4 <= memory4[read_index];
                read_index <= read_index + 1;
            end else begin
                // Reset outputs once all memory values are output
                out1 <= 0;
                out2 <= 0;
                out3 <= 0;
                out4 <= 0;
            end
        end 
    end

endmodule

module Activation_Normalization_Unit4x4(
    func_select,
    in1, in2, in3, in4,
    out1, out2, out3, out4
);
    parameter BIT_WIDTH = 8;
    input [2:0] func_select;
    input [15:0] in1, in2, in3, in4;  // 16-bit inputs
    output reg [7:0] out1, out2, out3, out4; // 8-bit outputs
    // Intermediate signals for the activation unit output
    reg [15:0] act_out1, act_out2, act_out3, act_out4;

    // Activation logic (combinational)
    always @(*) begin
        case (func_select)
            2'b01: begin
                // Placeholder for sigmoid logic (FUTURE WORK)
                act_out1 = in1;
                act_out2 = in2;
                act_out3 = in3;
                act_out4 = in4;
            end
            2'b10: begin
                // Placeholder for SoftMAX logic (FUTURE WORK)
                act_out1 = in1;
                act_out2 = in2;
                act_out3 = in3;
                act_out4 = in4;
            end
            2'b11: begin
                // Placeholder for other activation logic (FUTURE WORK)
                act_out1 = in1;
                act_out2 = in2;
                act_out3 = in3;
                act_out4 = in4;
            end
            default: begin // ReLU logic
                act_out1 = (in1[15] == 0) ? in1 : 16'b0; // Check MSB for sign
                act_out2 = (in2[15] == 0) ? in2 : 16'b0;
                act_out3 = (in3[15] == 0) ? in3 : 16'b0;
                act_out4 = (in4[15] == 0) ? in4 : 16'b0;
            end
        endcase
    end

    // Normalization logic (combinational)
    // Shift the 16-bit activated outputs to produce normalized 8-bit outputs
    always @(*) begin
//        //----------Showing the system is funtional----------//
//        out1 = act_out1[7:0];
//        out2 = act_out2[7:0];
//        out3 = act_out3[7:0];
//        out4 = act_out4[7:0];
        //----------Correct Shift right 8----------//
        out1 = act_out1 >> 8;
        out2 = act_out2 >> 8;
        out3 = act_out3 >> 8;
        out4 = act_out4 >> 8;
    end

endmodule

//------------------------------Controller related------------------------------//
module Controller(
    IR_out, IR_wr, IR_rd, // for IR_Mem
    IR_inc, IR_clr, // for IR_counter
    UB_addr, UB_rd, UB_wr, UB_counter_rst, // for Unified Buffer
    sds_counter_rst, sds_work, // for systolic data setup
    Weight_DDR_addr, Weight_rd, Weight_wr, // for Weight DDR3 memory
    Weight_interface_pushtime, Weight_interface_rst, Weight_interface_push, Weight_interface_pop, pop_complete, // for Weight interface
    Weight_FIFO_rst, Weight_FIFO_wr_en, Weight_FIFO_rd_en,
    Weight_FIFO_buf_empty, Weight_FIFO_buf_full, Weight_FIFO_fifo_counter, // for Weight FIFO
    Activation_FIFO_rst, Activation_FIFO_wr_en, Activation_FIFO_rd_en, 
    Activation_FIFO_buf_empty, Activation_FIFO_buf_full, Activation_FIFO_fifo_counter, // for Activation FIFO
    MMU_size, write_enable, read_enable, Accumulator_rst, accumulator_finish_storing, // for Accumulator
    func_select, // for Activation_Normalizaion_Unit
    MMU_rst, MMU_w_pass, MMU_weightload_complete, MMU_s_in1, MMU_s_in2, MMU_s_in3, MMU_s_in4, // MMU_weightload_complete is an 'input' to detect whether the weights are finishing loading into MMU
    MMU_r1_en, MMU_r2_en, MMU_r3_en, MMU_r4_en, MMU_c1_en, MMU_c2_en, MMU_c3_en, MMU_c4_en, 
    MMU_rowsignal1, MMU_rowsignal2, MMU_rowsignal3, MMU_rowsignal4, // for MMU
    clk, rst);
    
    parameter Instruction_size = 16;
    parameter address_size = 4;
    parameter MMUsize = 8;
    parameter S_initial = 0, S_fetch = 1; // fetch instructions for IR (in TPU)
    parameter S_decode = 2; // decode the instruction from IR
    // Weight load //
    parameter S_Weight_DDR3_to_interface1 = 3, S_Weight_DDR3_to_interface2 = 4, S_Weight_wait_stack_for_one_more_cycle = 5;
    parameter S_Weight_interface_to_WFIFO_stage1 = 6, S_Weight_interface_to_WFIFO_stage2 = 7, S_Weight_interface_to_WFIFO = 8, S_Weight_wait_FIFO_for_one_more_cycle = 9;
    parameter S_Weight_WFIFO_to_MMU = 10, S_Weight_MMU_rowdetect_stage1 = 11, S_Weight_MMU_rowdetect_stage2 = 12, S_Weight_MMU_load_complete = 13;
    parameter S_UB_to_sds1 = 14, S_UB_to_sds2 = 15, S_UB_to_sds3 = 16, S_Activation_Wait_for_one_more_cycle = 17, S_Activation_AFIFO_to_MMU_and_computing = 18;
    parameter S_MMU_computing = 19;
    parameter S_UB_store1 = 20, S_UB_store2 = 21;
    parameter S_Computing_Time_Compare = 22;
    parameter S_stay = 23;
    // opcode //
    parameter Read_Host_Memory = 0; //-----------------------   future work   -----------------------//
    parameter Read_Weight = 1; // weight_load (to MMU)
    parameter Computation = 2; // input from UB to A_FIFO to MMU then store in Accumulator (Called MatrixMultiply/Convolve in TPU paper)
    parameter Activate = 3; // values from accumulator through Activation-Normalization Unit and back to UB (Called Activate in TPU paper)
    parameter Write_Host_Memory = 4; //-----------------------   future work   -----------------------//
    // probably need to have another sub-state to do loop computation? like 1000 iterations??
//    parameter All_value_load = 0, MMU_compute = 1; // Maybe need other opcode? We dont have interaction with 'HOST MEMORY', but maybe need to extend the Accumulator to also make it as a local register file.
    
    // for IR_Mem
    input [Instruction_size-1:0]IR_out; 
    output reg IR_wr, IR_rd; 
    // for IR_counter
    output reg IR_inc, IR_clr;
    // for Unified Buffer
    output reg [address_size-1:0] UB_addr; // for 16 address locations
    output reg UB_rd, UB_wr, UB_counter_rst;
    // for systolic data setup
    output reg sds_counter_rst;
    output reg sds_work;
    // for Weight DDR3 memory
    output reg [address_size:0] Weight_DDR_addr;
    output reg Weight_rd, Weight_wr;
    // for Weight interface
    output reg [4:0] Weight_interface_pushtime; // for stack inside Weight interface
    output reg Weight_interface_rst;
    output reg Weight_interface_push, Weight_interface_pop;
    input pop_complete;
    // for Weight FIFO
    output reg Weight_FIFO_rst, Weight_FIFO_wr_en, Weight_FIFO_rd_en;
    input Weight_FIFO_buf_empty, Weight_FIFO_buf_full;
    input [2*address_size-1:0] Weight_FIFO_fifo_counter;
    // for Activation FIFO
    output reg Activation_FIFO_rst, Activation_FIFO_wr_en, Activation_FIFO_rd_en; 
    input Activation_FIFO_buf_empty, Activation_FIFO_buf_full;
    input [2*address_size-1:0] Activation_FIFO_fifo_counter;
    // for Accumulator
    output reg [MMUsize-1:0] MMU_size;
    output reg write_enable, read_enable, Accumulator_rst; 
    input accumulator_finish_storing;
    // for Activation_Normalizaion_Unit
    output reg [2:0] func_select; // for 4 different activation functions (fix in 00 for ReLU now and more functions are future work)
    // for MMU
    output reg MMU_rst, MMU_w_pass;
    input MMU_weightload_complete;
    output reg [15:0] MMU_s_in1, MMU_s_in2, MMU_s_in3, MMU_s_in4; // fix at 0
    output reg MMU_r1_en, MMU_r2_en, MMU_r3_en, MMU_r4_en, MMU_c1_en, MMU_c2_en, MMU_c3_en, MMU_c4_en;
    output reg [7:0] MMU_rowsignal1, MMU_rowsignal2, MMU_rowsignal3, MMU_rowsignal4;
    // general signal
    input clk, rst;
    
    reg [4:0] state, next_state;
    wire [Instruction_size-1:0]IR_out;
    wire [3:0] opcode = IR_out[15:12];
    wire [2:0] funtion_select = IR_out[11:9];
    reg [3:0] itr;
    reg [4:0] Weight_addr_count;
    reg [4:0] UB_addr_count;
    
    // Sequential logic to manage Weight_addr_count for each clock cycle in S_Weight_load_loop
    always @(posedge clk or posedge rst) begin
    if (rst) begin
        Weight_addr_count <= 5'b00000;
    end else if ((next_state == S_Weight_DDR3_to_interface1 || next_state == S_Weight_DDR3_to_interface2) && Weight_addr_count < 16) begin
        Weight_addr_count <= Weight_addr_count + 5'b00001;
    end
end
    
    // Sequential logic to manage UB_addr_count for each clock cycle in S_UB_loop (rd and wr)
    always @(posedge clk or posedge rst) begin
    if (rst) begin
        UB_addr_count <= 5'b00000;
    end else if (next_state == S_Activation_Wait_for_one_more_cycle || next_state == S_UB_to_sds1) begin
        UB_addr_count <= 5'b00000;
    end else if ((next_state == S_UB_to_sds2 || next_state == S_UB_to_sds3 || next_state == S_UB_store1 || next_state == S_UB_store2) && UB_addr_count < 8) begin
        UB_addr_count <= UB_addr_count + 5'b00001;
    end
end
    
    always@(IR_out) begin // maybe for computation more than once, 2 times or even a loop
        if (IR_out[15:12] == 2) begin
        itr <= IR_out[3:0]; // if opcode = 2 (computation, if itr >= 0, then loop back to computation state again)
        end
        else begin
        itr <= itr;    
        end
    end
    
    always@(posedge clk or posedge rst)begin: State_transitions
        if(rst == 1) state <= S_initial; else state <= next_state;
    end
    
    always@(state or opcode or UB_addr or Weight_DDR_addr or Weight_interface_pushtime or MMU_size or accumulator_finish_storing or func_select or MMU_weightload_complete or pop_complete or itr)begin 
        IR_wr = 0; IR_rd = 0; IR_inc = 0; IR_clr = 0; UB_addr = 4'b0000;
        UB_rd =0; UB_wr = 0; UB_counter_rst = 0; sds_counter_rst = 0;
        Weight_DDR_addr = 5'b0000; Weight_rd = 0; Weight_wr = 0; Weight_interface_pushtime = 5'b00000; Weight_interface_push = 0; Weight_interface_pop = 0;
        Weight_interface_rst = 0; Weight_FIFO_rst = 0; Weight_FIFO_wr_en = 0; Weight_FIFO_rd_en = 0;
        Activation_FIFO_rst = 0; Activation_FIFO_wr_en = 0; Activation_FIFO_rd_en = 0;
        MMU_size = 8'b00000000; write_enable = 0; read_enable = 0; Accumulator_rst = 0;
        func_select = 3'b000; MMU_rst = 0; MMU_w_pass = 0;
        MMU_s_in1 = 16'b0000000000000000; MMU_s_in2 = 16'b0000000000000000; MMU_s_in3 = 16'b0000000000000000; MMU_s_in4 = 16'b0000000000000000;
        MMU_r1_en = 0; MMU_r2_en = 0; MMU_r3_en = 0; MMU_r4_en = 0; MMU_c1_en = 0; MMU_c2_en = 0; MMU_c3_en = 0; MMU_c4_en = 0;
        MMU_rowsignal1 = 8'b00000000; MMU_rowsignal2 = 8'b00000000; MMU_rowsignal3 = 8'b00000000; MMU_rowsignal4 = 8'b00000000;
        next_state = state;
        
        case (state)
            S_initial    :begin
                         next_state = S_fetch;
                         IR_clr = 1;
                         // maybe dont need to reset
                         UB_counter_rst = 1; 
                         sds_counter_rst = 1;
                         Weight_interface_rst = 1;
                         Weight_FIFO_rst = 1;
                         Activation_FIFO_rst = 1;
                         Accumulator_rst = 1;
                         MMU_rst = 1;
                         $display("State: S_initial");
                         end
            S_fetch      :begin
                         next_state = S_decode;
                         IR_clr = 0;
//                         IR_inc = 1;
                         IR_rd = 1;
                         $display("State: S_fetch");
                         end
            S_decode     :begin
                            $display("opcode: %b", opcode);
                            case(opcode)
                            
                            // opcode = 0
                            Read_Host_Memory: begin //-----------------------   future work   -----------------------//
                            next_state = S_fetch; // fetch next instruction
                            IR_inc = 1;
                            $display("State: Read_Host_Memory (future work)"); //-----------------------   future work   -----------------------//
                            end //-----------------------   future work   -----------------------//
                            
                            // opcode = 1
                            Read_Weight: begin 
                            Weight_interface_pushtime = 5'b11111; // prepare for the weight input from DDR3
                            // data read out from Weight DDR
                            Weight_rd = 1; 
                            Weight_interface_push = 1;                             
                            Weight_DDR_addr = Weight_addr_count;
                            next_state = S_Weight_DDR3_to_interface1; // Move to the loop state to begin loading weights 
                            $display("State: Read_Weight");
                            end
                            
                            // opcode = 2
                            Computation: begin
                   //----------Use 1st value stored in UB_mem[0] for example----------// 
                            // UB data1 from mem[0] to UB output_reg                          
                            UB_rd = 1;
                            UB_wr = 0;
                            UB_counter_rst = 1; // reset UB_counter i (in UB)
                   //----------Should not allow these two signal, or it will load 8'bx into sds_reg and AFIFO----------// 
//                            sds_work = 1;
//                            Activation_FIFO_rd_en = 1;
                            next_state = S_UB_to_sds1;
                            $display("State: Computation");
                            end
                            
                            //opcode = 3
                            Activate: begin
                            read_enable = 1; // Accumulator read_enable = 1
                            func_select = 3'b000;
                            UB_counter_rst = 1; // reset UB_counter i (in UB)
                            next_state = S_UB_store1;
                            $display("State: Activate");
                            end
                            
                            // opcode = 4
                            Write_Host_Memory: begin //-----------------------   future work   -----------------------//
                            next_state = S_fetch; // fetch next instruction
                            IR_inc = 1;
                            $display("State: Write_Host_Memory (future work)"); //-----------------------   future work   -----------------------//
                            end //-----------------------   future work   -----------------------//
                                                  
                            default:
                                begin next_state = S_stay;
                                $display("Invalid opcode");
                                end
                            endcase
                         end
                         
            S_Weight_DDR3_to_interface1: begin
                                if (Weight_addr_count <= 15) begin
                                    $display("State: Weight_load_to_WInterface. . .");
                                    // data read out from Weight DDR
                                    Weight_rd = 1; 
                                    // Weight interface push
                                    Weight_interface_push = 1;                             
                                    Weight_DDR_addr = Weight_addr_count;                      
                                    next_state = S_Weight_DDR3_to_interface2; // Remain in the loop until count reaches 16
                                end else begin
                                    // All weights have been loaded into the FIFO
                                    $display("State: Weight_load_to_WInterface. . .");
                                    Weight_rd = 0;  
                                    Weight_interface_push = 0; 
                                    Weight_addr_count <= 5'b00000;
                                    next_state = S_Weight_wait_stack_for_one_more_cycle; // Proceed to the next state
                                end
                            end
            S_Weight_DDR3_to_interface2: begin
                            if (Weight_addr_count <= 15) begin
                                    $display("State: Weight_load_to_WInterface. . .");
                                    // data read out from Weight DDR
                                    Weight_rd = 1; 
                                    // Weight interface push
                                    Weight_interface_push = 1;                             
                                    Weight_DDR_addr = Weight_addr_count;                      
                                    next_state = S_Weight_DDR3_to_interface1; // Remain in the loop until count reaches 16
                                end
                                else begin
                                    // All weights have been loaded into the FIFO
                                    $display("State: Weight_load_to_WInterface. . .");
                                    Weight_rd = 0;  
                                    Weight_interface_push = 0; 
                                    next_state = S_Weight_wait_stack_for_one_more_cycle; // Proceed to the next state
                                end
                            end
            S_Weight_wait_stack_for_one_more_cycle: begin
                            $display("State: Weight completely loaded into WInterface ! ! !");
                            Weight_interface_pop = 1; // have to presend this pop signal one state earlier, or, all FIFOs[0] will load output = 0.
                            next_state = S_Weight_interface_to_WFIFO_stage1;
                            end 
            S_Weight_interface_to_WFIFO_stage1: begin
                            if(pop_complete != 1) begin
                            Weight_interface_pop = 1;
                            Weight_FIFO_wr_en = 1;
                            $display("State: Weight_load_to_WFIFO. . .");
                            next_state = S_Weight_interface_to_WFIFO_stage2;
                            end
                            else begin
                            next_state = S_Weight_wait_FIFO_for_one_more_cycle;
                            end
                            end
             S_Weight_interface_to_WFIFO_stage2: begin
                            if(pop_complete != 1) begin
                            Weight_interface_pop = 1;
                            Weight_FIFO_wr_en = 1;
                            $display("State: Weight_load_to_WFIFO. . .");
                            next_state = S_Weight_interface_to_WFIFO_stage1;
                            end
                            else begin
                            next_state = S_Weight_wait_FIFO_for_one_more_cycle;
                            end
                            end
            S_Weight_wait_FIFO_for_one_more_cycle: begin
                            $display("State: Weight completely loaded into WFIFO");
                            Weight_FIFO_wr_en = 0;
                            Weight_interface_pop = 0;
                            next_state = S_Weight_WFIFO_to_MMU;
                            end                 
            S_Weight_WFIFO_to_MMU: begin
                            $display("State: Weight_load_to_MMU");
                            Weight_FIFO_rd_en = 1;
                            MMU_w_pass = 1;
                            MMU_rowsignal1 = 8'b00000001; // should be 1 for correct counting
                            MMU_rowsignal2 = 8'b00000001;
                            MMU_rowsignal3 = 8'b00000010;
                            MMU_rowsignal4 = 8'b00000011;
                            next_state = S_Weight_MMU_rowdetect_stage1;
                            end   
            S_Weight_MMU_rowdetect_stage1: begin
                         $display("State: MMU row detect s1. . .");
                         Weight_FIFO_rd_en = 1;
                         MMU_w_pass = 1;
                         if (MMU_weightload_complete == 1) begin
                             next_state = S_Weight_MMU_load_complete; // Fetch next instruction when weights are fully loaded
                             Weight_FIFO_rd_en = 0;
                             MMU_w_pass = 0;
                         end else begin
                             next_state = S_Weight_MMU_rowdetect_stage2; // Continue weight loading
                         end
                         end
            S_Weight_MMU_rowdetect_stage2: begin
                         $display("State: MMU row detect s2. . .");
                         Weight_FIFO_rd_en = 1;
                         MMU_w_pass = 1;
                         if (MMU_weightload_complete == 1) begin
                             next_state = S_Weight_MMU_load_complete; // Fetch next instruction when weights are fully loaded
                             Weight_FIFO_rd_en = 0;
                             MMU_w_pass = 0;
                         end else begin
                             next_state = S_Weight_MMU_rowdetect_stage1; // Continue weight loading
                         end
                         end 
            S_Weight_MMU_load_complete: begin
                         $display("State: Weight complete loading into MMU");
                         next_state = S_fetch;
                         IR_inc = 1;
                         Weight_FIFO_rst = 1; // clear Weight FIFO in case of contain useless weight values
                         end             
            S_UB_to_sds1: begin
                         $display("State: Activation_load_to_AFIFO s1. . .");
                //----------Use 1st value stored in UB_mem[0] for example----------// 
                         // Other data be sent to output_reg's'                                     
                         UB_rd = 1;
                         UB_wr = 0;
                //----------Should allow sds_work = 1 for Systolic Data Setup----------//
                         // UB data1 from UB output_reg to sds output1(reg)
                         sds_work = 1;
                //----------However, should not allow Activation_FIFO_wr_en = 1 or it will load 8'dx into AFIFO----------//
//                         Activation_FIFO_wr_en = 1;
                         next_state = S_UB_to_sds2;               
                         end
            S_UB_to_sds2: begin
                         if (UB_addr_count <= 7) begin
                         $display("State: Activation_load_to_AFIFO s2. . .");
                //----------Use 1st value stored in UB_mem[0] for example----------//
                         // Other data be sent to output_reg's'
                         UB_rd = 1;
                         UB_wr = 0;
                         // Other data be sent to sds_output_reg's'
                         sds_work = 1;
                         // UB data1 from sds output1(reg) to AFIFO1[0]
                         Activation_FIFO_wr_en = 1;
                         next_state = S_UB_to_sds3; // need this extra state for signal detection
                         end else begin
                             next_state = S_Activation_Wait_for_one_more_cycle;
                         end                   
                         end
            S_UB_to_sds3: begin
                         if (UB_addr_count <= 7) begin
                         $display("State: Activation_load_to_AFIFO s3. . .");
                         UB_rd = 1;
                         UB_wr = 0;
                         sds_work = 1;
                         Activation_FIFO_wr_en = 1;
                         next_state = S_UB_to_sds2;
                         end else begin
                             next_state = S_Activation_Wait_for_one_more_cycle;
                         end                   
                         end
            S_Activation_Wait_for_one_more_cycle: begin
                         $display("State: Activation completely loaded into AFIFO");
                         UB_rd = 0;
                         UB_wr = 0;
                         sds_work = 0;
                         Activation_FIFO_wr_en = 0;
                         next_state = S_Activation_AFIFO_to_MMU_and_computing;
                   
//                   //----------------test purpose 1118 1:30 finished----------------//
//                         next_state = S_fetch;
//                         IR_inc = 1;
                         end
            S_Activation_AFIFO_to_MMU_and_computing: begin
                         $display("State: Activation start loading into MMU");
                         Activation_FIFO_rd_en = 1;
                         MMU_s_in1 = 16'b0000000000000000; 
                         MMU_s_in2 = 16'b0000000000000000; 
                         MMU_s_in3 = 16'b0000000000000000; 
                         MMU_s_in4 = 16'b0000000000000000;
                         MMU_r1_en = 1; MMU_r2_en = 1; MMU_r3_en = 1; MMU_r4_en = 1; 
                         MMU_c1_en = 1; MMU_c2_en = 1; MMU_c3_en = 1; MMU_c4_en = 1;
                         MMU_size = 8'b00000100;
                         write_enable = 1; // Accumulator write_enable = 1
                         next_state = S_MMU_computing;
                         end  
            S_MMU_computing: begin
                         if (accumulator_finish_storing != 1) begin
                         $display("State: MMU Computing. . .");
                         Activation_FIFO_rd_en = 1;
                         MMU_s_in1 = 16'b0000000000000000; 
                         MMU_s_in2 = 16'b0000000000000000; 
                         MMU_s_in3 = 16'b0000000000000000; 
                         MMU_s_in4 = 16'b0000000000000000;
                         MMU_r1_en = 1; MMU_r2_en = 1; MMU_r3_en = 1; MMU_r4_en = 1; 
                         MMU_c1_en = 1; MMU_c2_en = 1; MMU_c3_en = 1; MMU_c4_en = 1;
                         MMU_size = 8'b00000100;
                         write_enable = 1; // Accumulator write_enable = 1
                         next_state = S_MMU_computing;
                         end
                         else begin
//----------------Finish Computing and values are stored in Accumulator and waiting for Activation Function instruction----------------//
                         $display("State: Values finishing loading into Accumulator!!!");
                         Activation_FIFO_rd_en = 0;
                         MMU_r1_en = 0; MMU_r2_en = 0; MMU_r3_en = 0; MMU_r4_en = 0; 
                         MMU_c1_en = 0; MMU_c2_en = 0; MMU_c3_en = 0; MMU_c4_en = 0;
                         write_enable = 0; // Accumulatpor write_enable = 0
                         next_state = S_fetch; // fetch for Activate (ReLU, Sigmoid, SoftMax...)
                         IR_inc = 1;
                         end
                         end                        
            S_UB_store1: begin                   
                         $display("State: UB storing s1. . .");
                         if (UB_addr_count <= 4) begin
                         UB_rd = 0;
                         UB_wr = 1;
                         read_enable = 1; // Accumulator read_enable = 1
                         func_select = 3'b000;
                         next_state = S_UB_store2;   
                         end
                         else begin
                         $display("State: Values finishing storing back to Unified Buffer");
                         UB_rd = 0;
                         UB_wr = 0;
                         read_enable = 0; // Accumulator read_enable = 0
                         next_state = S_Computing_Time_Compare; 
                         end
                         end
            S_UB_store2: begin
                         $display("State: UB storing s2. . .");
                         if (UB_addr_count <= 4) begin
                         UB_rd = 0;
                         UB_wr = 1;
                         read_enable = 1; // Accumulator read_enable = 1
                         func_select = 3'b000;
                         next_state = S_UB_store1;   
                         end
                         else begin
                         $display("State: Values finishing storing back to Unified Buffer");
                         UB_rd = 0;
                         UB_wr = 0;
                         read_enable = 0; // Accumulator read_enable = 0

                         next_state = S_Computing_Time_Compare; 
                         end
                         end
            S_Computing_Time_Compare: begin
            //----------future work----------//
//                         $display("State: Checking iteration. . .");
//                         if (itr > 0) begin
//                         $display("State: Compute for the next itration!!!");
//                         itr = itr - 1;
//                         UB_rd = 1;
//                         UB_wr = 0;
//                         UB_counter_rst = 1; // reset UB_counter i (in UB)
//                         next_state = S_UB_to_sds1; 
//                         end
//                         else begin
                         $display("State: Finish all the computation");
                         next_state = S_fetch; 
                         IR_inc = 1;
                         end
//                         end
            S_stay       :begin
                         IR_rd = 1;                 
                         $display("State: S_stay");
                         end
            default: begin next_state = S_initial;
            $display("State: Default, resetting to S_initial");
            end
        endcase
    end
endmodule


module TPU4X4(clk, rst);
    input clk, rst;
    
    parameter Instruction_size = 16;
    parameter address_size = 4;
    parameter MMUsize = 8;
    parameter Data_size = 8;
    wire [Instruction_size-1:0]IR_out; 
    wire IR_wr, IR_rd; 
    // for IR_counter
    wire [2*address_size-1:0] IR_addr;
    wire IR_inc, IR_clr;
    // for Unified Buffer
    wire [address_size-1:0] UB_addr; // for 16 address locations
    wire UB_rd, UB_wr, UB_counter_rst;
    wire [Data_size-1:0] UB_dataout1, UB_dataout2, UB_dataout3, UB_dataout4, UB_dataout5, UB_dataout6, UB_dataout7, UB_dataout8;
    wire [Data_size-1:0] UB_dataout9, UB_dataout10, UB_dataout11, UB_dataout12, UB_dataout13, UB_dataout14, UB_dataout15, UB_dataout16;
    // for systolic data setup
    wire sds_counter_rst, sds_work;
    wire [Data_size-1:0] Activation_sds_to_AFIFO1, Activation_sds_to_AFIFO2, Activation_sds_to_AFIFO3, Activation_sds_to_AFIFO4;
    // for Weight DDR3 memory
    wire [address_size:0] Weight_DDR_addr;
    wire Weight_rd, Weight_wr;
    wire [Data_size-1:0] Weight_DDR3_to_Interface;
    // for Weight interface
    wire [4:0] Weight_interface_pushtime; // for stack inside Weight interface
    wire Weight_interface_push, Weight_interface_pop;
    wire pop_complete;
    wire Weight_interface_rst;
    wire [Data_size-1:0] Weight_Interface_to_FIFO1, Weight_Interface_to_FIFO2, Weight_Interface_to_FIFO3, Weight_Interface_to_FIFO4;
    // for Weight FIFO
    wire Weight_FIFO_rst, Weight_FIFO_wr_en, Weight_FIFO_rd_en;
    wire Weight_FIFO_buf_empty, Weight_FIFO_buf_full;
    wire [2*address_size-1:0] Weight_FIFO_fifo_counter;
    wire [Data_size-1:0] Weight_FIFO_to_MMU1, Weight_FIFO_to_MMU2, Weight_FIFO_to_MMU3, Weight_FIFO_to_MMU4;
    // for Activation FIFO
    wire Activation_FIFO_rst, Activation_FIFO_wr_en, Activation_FIFO_rd_en; 
    wire Activation_FIFO_buf_empty, Activation_FIFO_buf_full;
    wire [2*address_size-1:0] Activation_FIFO_fifo_counter;
    wire [Data_size-1:0] Activation_FIFO_to_MMU1, Activation_FIFO_to_MMU2, Activation_FIFO_to_MMU3, Activation_FIFO_to_MMU4;
    // for Accumulator
    wire [MMUsize-1:0] MMU_size;
    wire write_enable, read_enable, Accumulator_rst; 
    wire accumulator_finish_storing;
    wire [15:0] Accumulator_to_AN_out1, Accumulator_to_AN_out2, Accumulator_to_AN_out3, Accumulator_to_AN_out4;
    // for Activation_Normalizaion_Unit
    wire [2:0] func_select; // for 4 different activation functions (fix in 00 for ReLU now and more functions are future work)
    wire [Data_size-1:0] AN_to_UB1, AN_to_UB2, AN_to_UB3, AN_to_UB4;
    // for MMU
    wire MMU_rst, MMU_w_pass;
    wire MMU_weightload_complete;
    wire [15:0] MMU_s_in1, MMU_s_in2, MMU_s_in3, MMU_s_in4; // fix at 0
    wire MMU_r1_en, MMU_r2_en, MMU_r3_en, MMU_r4_en, MMU_c1_en, MMU_c2_en, MMU_c3_en, MMU_c4_en;
    wire [7:0] MMU_rowsignal1, MMU_rowsignal2, MMU_rowsignal3, MMU_rowsignal4;
    wire [15:0] MMU_to_Accumulator_out1, MMU_to_Accumulator_out2, MMU_to_Accumulator_out3, MMU_to_Accumulator_out4;
    
    
    
    //------------------------------IR related------------------------------//
    IR_Mem IR_Mem1(.R_data(IR_out), .W_data(), .addr(IR_addr), .clk(clk), .IR_rd(IR_rd), .IR_wr());
    IR_counter IR_counter1(.IR_addr(IR_addr), .IR_inc(IR_inc), .IR_clr(IR_clr), .clk(clk));
    //------------------------------Weight related------------------------------//
    Weight_DDR3 WDDR3_1(.R_data(Weight_DDR3_to_Interface), .W_data(), .addr(Weight_DDR_addr), .clk(clk), .rd(Weight_rd), .wr());
    Weight_interface Winterface1(.clk(clk), .reset(Weight_interface_rst), .weight_in(Weight_DDR3_to_Interface), .push_time(Weight_interface_pushtime), .push(Weight_interface_push), .pop(Weight_interface_pop), .pop_complete(pop_complete), .out1(Weight_Interface_to_FIFO1), .out2(Weight_Interface_to_FIFO2), .out3(Weight_Interface_to_FIFO3), .out4(Weight_Interface_to_FIFO4));
    Weight_FIFO WFIFO1(.clk(clk), .rst(Weight_FIFO_rst), .buf_in(Weight_Interface_to_FIFO1), .buf_out(Weight_FIFO_to_MMU1), .wr_en(Weight_FIFO_wr_en), .rd_en(Weight_FIFO_rd_en), .buf_empty(), .buf_full(), .fifo_counter());
    Weight_FIFO WFIFO2(.clk(clk), .rst(Weight_FIFO_rst), .buf_in(Weight_Interface_to_FIFO2), .buf_out(Weight_FIFO_to_MMU2), .wr_en(Weight_FIFO_wr_en), .rd_en(Weight_FIFO_rd_en), .buf_empty(), .buf_full(), .fifo_counter());
    Weight_FIFO WFIFO3(.clk(clk), .rst(Weight_FIFO_rst), .buf_in(Weight_Interface_to_FIFO3), .buf_out(Weight_FIFO_to_MMU3), .wr_en(Weight_FIFO_wr_en), .rd_en(Weight_FIFO_rd_en), .buf_empty(), .buf_full(), .fifo_counter());
    Weight_FIFO WFIFO4(.clk(clk), .rst(Weight_FIFO_rst), .buf_in(Weight_Interface_to_FIFO4), .buf_out(Weight_FIFO_to_MMU4), .wr_en(Weight_FIFO_wr_en), .rd_en(Weight_FIFO_rd_en), .buf_empty(), .buf_full(), .fifo_counter());
    //------------------------------MMU related------------------------------//
    MMU4x4 MMU1(.a1(Activation_FIFO_to_MMU1), .a2(Activation_FIFO_to_MMU2), .a3(Activation_FIFO_to_MMU3), .a4(Activation_FIFO_to_MMU4), .w1(Weight_FIFO_to_MMU1), .w2(Weight_FIFO_to_MMU2), .w3(Weight_FIFO_to_MMU3), .w4(Weight_FIFO_to_MMU4),
    .s_in1(MMU_s_in1), .s_in2(MMU_s_in2), .s_in3(MMU_s_in3), .s_in4(MMU_s_in4), .o1(MMU_to_Accumulator_out1), .o2(MMU_to_Accumulator_out2), .o3(MMU_to_Accumulator_out3), .o4(MMU_to_Accumulator_out4), .rst(MMU_rst), .clk(clk),
    .w_pass(MMU_w_pass), .weightload_complete(MMU_weightload_complete), .r1_en(MMU_r1_en), .r2_en(MMU_r2_en), .r3_en(MMU_r3_en), .r4_en(MMU_r4_en), .c1_en(MMU_c1_en), .c2_en(MMU_c2_en), .c3_en(MMU_c3_en), .c4_en(MMU_c4_en),
    .rowsignal1(MMU_rowsignal1), .rowsignal2(MMU_rowsignal2), .rowsignal3(MMU_rowsignal3), .rowsignal4(MMU_rowsignal4));
    //------------------------------Other Computation related------------------------------//
    //------------------------------before MMU------------------------------//
    unified_buffer UB1(.addr(UB_addr), .clk(clk), .rd(UB_rd), .wr(UB_wr), .counter_rst(UB_counter_rst), 
    .data_in1(AN_to_UB1), .data_in2(AN_to_UB2), .data_in3(AN_to_UB3), .data_in4(AN_to_UB4), 
    .data_out1(UB_dataout1), .data_out2(UB_dataout2), .data_out3(UB_dataout3), .data_out4(UB_dataout4), .data_out5(UB_dataout5), .data_out6(UB_dataout6), .data_out7(UB_dataout7), .data_out8(UB_dataout8), 
    .data_out9(UB_dataout9), .data_out10(UB_dataout10), .data_out11(UB_dataout11), .data_out12(UB_dataout12), .data_out13(UB_dataout13), .data_out14(UB_dataout14), .data_out15(UB_dataout15), .data_out16(UB_dataout16));
    sds4x4 sds1(.clk(clk), .counter_rst(sds_counter_rst), .sds_work(sds_work), .a0(UB_dataout1), .a1(UB_dataout2), .a2(UB_dataout3), .a3(UB_dataout4), .b0(UB_dataout5), .b1(UB_dataout6), .b2(UB_dataout7), .b3(UB_dataout8), 
    .c0(UB_dataout9), .c1(UB_dataout10), .c2(UB_dataout11), .c3(UB_dataout12), .d0(UB_dataout13), .d1(UB_dataout14), .d2(UB_dataout15), .d3(UB_dataout16), .output1(Activation_sds_to_AFIFO1), .output2(Activation_sds_to_AFIFO2), .output3(Activation_sds_to_AFIFO3), .output4(Activation_sds_to_AFIFO4));
    Activation_FIFO AFIFO1(.clk(clk), .rst(Activation_FIFO_rst), .buf_in(Activation_sds_to_AFIFO1), .buf_out(Activation_FIFO_to_MMU1), .wr_en(Activation_FIFO_wr_en), .rd_en(Activation_FIFO_rd_en), .buf_empty(), .buf_full(), .fifo_counter());
    Activation_FIFO AFIFO2(.clk(clk), .rst(Activation_FIFO_rst), .buf_in(Activation_sds_to_AFIFO2), .buf_out(Activation_FIFO_to_MMU2), .wr_en(Activation_FIFO_wr_en), .rd_en(Activation_FIFO_rd_en), .buf_empty(), .buf_full(), .fifo_counter());
    Activation_FIFO AFIFO3(.clk(clk), .rst(Activation_FIFO_rst), .buf_in(Activation_sds_to_AFIFO3), .buf_out(Activation_FIFO_to_MMU3), .wr_en(Activation_FIFO_wr_en), .rd_en(Activation_FIFO_rd_en), .buf_empty(), .buf_full(), .fifo_counter());
    Activation_FIFO AFIFO4(.clk(clk), .rst(Activation_FIFO_rst), .buf_in(Activation_sds_to_AFIFO4), .buf_out(Activation_FIFO_to_MMU4), .wr_en(Activation_FIFO_wr_en), .rd_en(Activation_FIFO_rd_en), .buf_empty(), .buf_full(), .fifo_counter());
    //------------------------------After MMU------------------------------//
    Accumulator4x4 Accumlator1(.MMU_size(MMU_size), .write_enable(write_enable), .read_enable(read_enable), .in1(MMU_to_Accumulator_out1), .in2(MMU_to_Accumulator_out2), .in3(MMU_to_Accumulator_out3), .in4(MMU_to_Accumulator_out4), 
    .out1(Accumulator_to_AN_out1), .out2(Accumulator_to_AN_out2), .out3(Accumulator_to_AN_out3), .out4(Accumulator_to_AN_out4), .accumulator_finish_storing(accumulator_finish_storing), .clk(clk), .rst(Accumulator_rst));
    Activation_Normalization_Unit4x4 AN1(.func_select(func_select), .in1(Accumulator_to_AN_out1), .in2(Accumulator_to_AN_out2), .in3(Accumulator_to_AN_out3), .in4(Accumulator_to_AN_out4), .out1(AN_to_UB1), .out2(AN_to_UB2), .out3(AN_to_UB3), .out4(AN_to_UB4));
    //------------------------------Controller related------------------------------//
    Controller controller1(
    .IR_out(IR_out), .IR_wr(), .IR_rd(IR_rd), // for IR_Mem
    .IR_inc(IR_inc), .IR_clr(IR_clr), // for IR_counter
    .UB_addr(UB_addr), .UB_rd(UB_rd), .UB_wr(UB_wr), .UB_counter_rst(UB_counter_rst), // for Unified Buffer
    .sds_counter_rst(sds_counter_rst), .sds_work(sds_work), // for systolic data setup
    .Weight_DDR_addr(Weight_DDR_addr), .Weight_rd(Weight_rd), .Weight_wr(), // for Weight DDR3 memory
    .Weight_interface_pushtime(Weight_interface_pushtime), .Weight_interface_rst(Weight_interface_rst), .Weight_interface_push(Weight_interface_push), .Weight_interface_pop(Weight_interface_pop), // for Weight interface
    .pop_complete(pop_complete), .Weight_FIFO_rst(Weight_FIFO_rst), .Weight_FIFO_wr_en(Weight_FIFO_wr_en), .Weight_FIFO_rd_en(Weight_FIFO_rd_en),
    .Weight_FIFO_buf_empty(), .Weight_FIFO_buf_full(), .Weight_FIFO_fifo_counter(), // for Weight FIFO
    .Activation_FIFO_rst(Activation_FIFO_rst), .Activation_FIFO_wr_en(Activation_FIFO_wr_en), .Activation_FIFO_rd_en(Activation_FIFO_rd_en), 
    .Activation_FIFO_buf_empty(), .Activation_FIFO_buf_full(), .Activation_FIFO_fifo_counter(), // for Activation FIFO
    .MMU_size(MMU_size), .write_enable(write_enable), .read_enable(read_enable), .Accumulator_rst(Accumulator_rst), .accumulator_finish_storing(accumulator_finish_storing), // for Accumulator
    .func_select(func_select), // for Activation_Normalizaion_Unit
    .MMU_rst(MMU_rst), .MMU_w_pass(MMU_w_pass), .MMU_weightload_complete(MMU_weightload_complete), .MMU_s_in1(MMU_s_in1), .MMU_s_in2(MMU_s_in2), .MMU_s_in3(MMU_s_in3), .MMU_s_in4(MMU_s_in4), // MMU_weightload_complete is an 'input' to detect whether the weights are finishing loading into MMU
    .MMU_r1_en(MMU_r1_en), .MMU_r2_en(MMU_r2_en), .MMU_r3_en(MMU_r3_en), .MMU_r4_en(MMU_r4_en), .MMU_c1_en(MMU_c1_en), .MMU_c2_en(MMU_c2_en), .MMU_c3_en(MMU_c3_en), .MMU_c4_en(MMU_c4_en), 
    .MMU_rowsignal1(MMU_rowsignal1), .MMU_rowsignal2(MMU_rowsignal2), .MMU_rowsignal3(MMU_rowsignal3), .MMU_rowsignal4(MMU_rowsignal4), // for MMU
    .clk(clk), .rst(rst));
endmodule
