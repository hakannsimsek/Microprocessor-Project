module program_counter(
	input fetch,
	output reg [1:0] addr
);
	reg [1:0] _count;
	initial
		_count = 0;
	always @(fetch)
	begin
		if ( _count == 4)
		begin
			_count = 0;
		end
		addr = _count;
		_count = _count + 1;
	end
endmodule

// Bunun adı rom denemek için böyle yaptım
module rom_kernel(
	input fetch,
	input [1:0] addr,
	output reg [11:0] instruction
);
	reg [11:0] instructions [3:0];
	initial begin
		$readmemh("rom_init.txt", instructions);
	end
	assign instruction = instructions[addr];
endmodule

// Bu main gibi sırf proje adından dolayı böyle oldu
module rom(input fetch);
	wire [1:0] addr;
	wire [11:0] instruction;
	program_counter pc(fetch, addr[1:0]);
	rom_kernel romk(fetch, addr[1:0], instruction[11:0]);
endmodule
