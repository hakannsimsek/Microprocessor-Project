module intr(
    input clk,
	 output zf,of,cf,nf
);

    wire [1:0] signal;
	 wire [2:0] opcode;
	 wire [7:0] imm;
	 wire [7:0] result;
	 wire type;
	 wire readWrite;
	 wire [7:0] ramOut;
	 wire [7:0] acc;

	wire [1:0] addr;
	wire [11:0] instruction;

    control_unit cu(clk, signal[1:0]);
	 
	program_counter pc(~signal[0]&~signal[1], addr[1:0]);
	rom romk(addr[1:0], instruction[11:0]);
	
	 program_memory pm(signal[1:0],instruction[11:0],opcode[2:0],imm[7:0],type,readWrite);
	 alu alunit(imm[7:0],acc[7:0], opcode[2:0],result[7:0], zf,of,cf,nf);
	 accumulator accum(result[7:0],~signal[0]&signal[1]&~type,acc[7:0]);
	 
	 ram ram_unit(imm[7:0],acc[7:0],readWrite,signal[0]&signal[1]&type,ramOut[7:0]);




//module accumulator(
//        input [7:0] data,
//        input write,
//        input enable,
//        output reg [7:0] out
//    );

	 
	 
//	 module ram(
//        input [7:0] addr,
//        input [7:0] data,
//        input write,
//        input enable,
//        output [7:0] out
//    );
	 
	 
endmodule

module program_counter(
	input fetch,
	output reg [1:0] addr
);
	reg [1:0] _count;
	initial
		_count = 0;
	always @(posedge fetch)
	begin
		if ( _count == 4)
		begin
			_count = 0;
		end
		addr = _count;
		_count = _count + 1;
	end
endmodule

// Bunun ad? rom denemek i?in b?yle yapt?m
module rom(
	input [1:0] addr,
	output reg [11:0] instruction
);
	reg [11:0] instructions [3:0];
	initial begin
		$readmemh("rom_init.txt", instructions);
	end
	assign instruction = instructions[addr];
endmodule


module control_unit(
        input clk,
        output reg [1:0] signal
);
    initial
        signal = -1;
    always @(posedge clk)
    begin
        if (signal == 2'b11)
            signal = 2'b00;
        else
            signal = signal + 1'b1;
        
    end
endmodule

module program_memory(
	 input [1:0] signal,
    input [11:0] addr,
    output reg [2:0] opcode,
    output reg [7:0] imm,
	 output reg type,
	 output reg readWrite
);
    always @(signal)																
    begin
		if( signal == 2'b01 )
		begin
		  type = addr[11];
		  readWrite = addr[10];
        opcode = addr[10:8];
        imm = addr[7:0];
		 end
    end
endmodule





module alu(i0,i1, opcode,acc, zf,of,cf,nf);
    /* Inputs */
    input [7:0] i0;
    input [7:0] i1;
    input [2:0] opcode;
        //input [1:0] signal;
    /* Outputs */
    output [7:0] acc;
    output zf,cf,nf,of;
    /* Memories */
    reg [7:0] acc;
    reg zf,cf,nf,of;
    reg [8:0] virtualOutput;
    
    /**
     * Overflow cases
     * a) add
     *     1) p + p = n
     *        2) n + n = p
     * b) subtract
     *     1) p - n = n
     *     2) n - p = p
     */

    /* Listener */
    always@(*)
    begin
			
        /* Flags */
        zf= 1'b0;
        of= 1'b0;
        cf= 1'b0;
        nf= 1'b0;
        
        // Execute by opcode
			case(opcode)
					3'b000:  begin             // ADD: cf, zf, of, nf
													 virtualOutput = i0+i1;
										 cf = virtualOutput[8];
													if (i0[7] == 0 && i1[7] == 0 && virtualOutput[7] == 1) // p + p = n
														of = 1;
													if (i0[7] == 1 && i1[7] == 1 && virtualOutput[7] == 0) // n + n = p
														of = 1;
										acc = i1+i0;
									end
					3'b001:  begin            // SUB: cf, zf, of, nf
										 virtualOutput = i1-i0;
										 cf = virtualOutput[8];
							  if (i0[7] == 0 && i1[7] == 1 && virtualOutput[7] == 1) // p - n = n
										of = 1;
							  if (i0[7] == 1 && i1[7] == 0 && virtualOutput[7] == 0) // n - p = p
										of = 1;
								acc = i1-i0;
									end
					3'b010:  begin                // AND: zf, nf
										 acc = i0&i1;
									end
					3'b011:  begin                // MOV: zf, nf
										 acc = i0;
									end
					3'b100:  begin               // SHFL: zf, nf 
										 acc = i0<<1;
									end
					3'b101:  begin            // SHFR: zf, nf
										 acc = i0>>1;
									end
					3'b110:  begin            // OR: zf, nf
										 acc = i0|i1;
									end
					3'b111:  begin            // NOT: zf, nf
										 acc = (~i1);
									end
			endcase
			  // If all output values are zero, make zero flag one, otherwise zero
			if( acc[0] | acc[1] | acc[2] | acc[3] | acc[4] | acc[5] | acc[6] | acc[7] )
					zf= 1'b0;
			else
					zf= 1'b1;
			  // Negative flag is based on the first bit    
			nf = acc[7];
		 end
	 

endmodule  

module accumulator(
        input [7:0] data,
        input enable,
        output reg [7:0] out
    );
    reg [7:0] cache;
    always @(posedge enable)
    begin
		if(enable)
			cache = data;
			out = cache;
	   end
endmodule



module ram(
        input [7:0] addr,
        input [7:0] data,
        input write,
        input enable,
        output [7:0] out
    );
    wire [7:0] intmd0, intmd1;

    decoder_2_bits dc(addr[7], enable0, enable1);
    memory_128_bits m0(addr[6:0], data, write, enable0&enable, intmd0);
    memory_128_bits m1(addr[6:0], data, write, enable1&enable, intmd1);
    mux_2_bits mux(addr[7], intmd0, intmd1, out);
endmodule

module memory_128_bits(
        input [6:0] addr,
        input [7:0] data,
        input write,
        input enable,
        output [7:0] out
    );
    wire [7:0] intmd0, intmd1;

    decoder_2_bits dc(addr[6], enable0, enable1);
    memory_64_bits m0(addr[5:0], data, write, enable0&enable, intmd0);
    memory_64_bits m1(addr[5:0], data, write, enable1&enable, intmd1);
    mux_2_bits mux(addr[6], intmd0, intmd1, out);
endmodule

module memory_64_bits(
        input [5:0] addr,
        input [7:0] data,
        input write,
        input enable,
        output [7:0] out
    );
    wire [7:0] intmd0, intmd1;

    decoder_2_bits dc(addr[5], enable0, enable1);
    memory_32_bits m0(addr[4:0], data, write, enable0&enable, intmd0);
    memory_32_bits m1(addr[4:0], data, write, enable1&enable, intmd1);
    mux_2_bits mux(addr[5], intmd0, intmd1, out);
endmodule

module memory_32_bits(
        input [4:0] addr,
        input [7:0] data,
        input write,
        input enable,
        output [7:0] out
    );
    wire [7:0] intmd0, intmd1;

    decoder_2_bits dc(addr[4], enable0, enable1);
    memory_16_bits m0(addr[3:0], data, write, enable0&enable, intmd0);
    memory_16_bits m1(addr[3:0], data, write, enable1&enable, intmd1);
    mux_2_bits mux(addr[4], intmd0, intmd1, out);
endmodule

module memory_16_bits(
        input [3:0] addr,
        input [7:0] data,
        input write,
        input enable,
        output [7:0] out
    );
    wire [7:0] intmd0, intmd1;

    decoder_2_bits dc(addr[3], enable0, enable1);
    memory_8_bits m0(addr[2:0], data, write, enable0&enable, intmd0);
    memory_8_bits m1(addr[2:0], data, write, enable1&enable, intmd1);
    mux_2_bits mux(addr[3], intmd0, intmd1, out);
endmodule

module memory_8_bits(
        input [2:0] addr,
        input [7:0] data,
        input write,
        input enable,
        output [7:0] out
    );
    wire [7:0] intmd0, intmd1;

    decoder_2_bits dc(addr[2], enable0, enable1);
    memory_4_bits m0(addr[1:0], data, write, enable0&enable, intmd0);
    memory_4_bits m1(addr[1:0], data, write, enable1&enable, intmd1);
    mux_2_bits mux(addr[2], intmd0, intmd1, out);
endmodule

module memory_4_bits(
        input [1:0] addr,
        input [7:0] data,
        input write,
        input enable,
        output [7:0] out
    );
    wire [7:0] intmd0, intmd1;

    decoder_2_bits dc(addr[1], enable0, enable1);
    memory_2_bits m0(addr[0], data, write, enable0&enable, intmd0);
    memory_2_bits m1(addr[0], data, write, enable1&enable, intmd1);
    mux_2_bits mux(addr[1], intmd0, intmd1, out);
endmodule

module memory_2_bits(
        input addr, 
        input [7:0] data,
        input write,
        input enable,
        output [7:0] out
    );
    wire [7:0] intmd0, intmd1;
    
    decoder_2_bits dc(addr, enable0, enable1);
    binary_cell bc0(data, write, enable0&enable, intmd0);
    binary_cell bc1(data, write, enable1&enable, intmd1);
    mux_2_bits mux(addr, intmd0, intmd1, out);
endmodule

module binary_cell(
        input [7:0] data,
        input write,
        input enable,
        output reg [7:0] out
    );
    reg [7:0] cache;
    always @(posedge enable)
    begin
        if (enable)
        begin
            if(write) cache = data;
            if(write == 0) out = cache;
        end
    end
endmodule

module mux_2_bits(
        input decider,
        input [7:0] in0,
        input [7:0] in1,
        output reg [7:0] out
    );
    always @(decider, in0, in1)
    begin
        out = decider == 1'b0 ? in0 : in1;
    end
endmodule

module decoder_2_bits(
        input in,
        output reg out0,
        output reg out1
    );
    always @(in)
    begin
        out0 = ~in;
        out1 = in;
    end
endmodule


