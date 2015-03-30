module snake(
//	Clock Input
  input CLOCK_50,	//	50 MHz
  input CLOCK_27,     //      27 MHz
//	Push Button
 inout [35:0] GPIO_0,GPIO_1,	//	GPIO Connections
//	TV Decoder
//TD_DATA,    	//	TV Decoder Data bus 8 bits
//TD_HS,		//	TV Decoder H_SYNC
//TD_VS,		//	TV Decoder V_SYNC
  output TD_RESET,	//	TV Decoder Reset
// VGA
  output VGA_CLK,   						//	VGA Clock
  output VGA_HS,							//	VGA H_SYNC
  output VGA_VS,							//	VGA V_SYNC
  output VGA_BLANK,						//	VGA BLANK
  output VGA_SYNC,						//	VGA SYNC
  output [9:0] VGA_R,   						//	VGA Red[9:0]
  output [9:0] VGA_G,	 						//	VGA Green[9:0]
  output [9:0] VGA_B,    //	VGA Blue[9:0]
  input [3:0] KEY,
  input [17:0] SW,
  output  [6:0]  HEX0, HEX1, HEX2, HEX3, HEX4, HEX5, HEX6, HEX7,
  output  [8:0]  LEDG,  //  LED Green[8:0]
  output  [17:0]  LEDR,  //  LED Red[17:0]
 
  input	PS2_DAT,
  input	PS2_CLK
);
	reg [9:0] xm = 10'd96;
	reg [9:0] ym = 10'd35;
	reg [1:0] snake_direction = 2'b00;
	wire [9:0] x = xm;
	wire [9:0] y = ym;
	wire [7:0] keyValue;
	wire [29999:0] snake_body;
	reg [5:0] new_head_x;
	reg [5:0] new_head_y;
	reg lose = 0;
	wire tick;
	reg [31:0] snake_length = 1;
	reg [11:0] in = 12'b011001011001;
	reg [2499:0] tail = 2499'b0;
	wire [5:0] randx;
	wire [5:0] randy;
	wire [11:0] nom;
	reg eat = 0;
	reg [31:0] counter = 0;
	reg [31:0] tps = 32'd15;
	
	DFlipFlop(tick, in, snake_body[11:0], 1);//head
	
	genvar i;
	generate
		for (i = 1; i < 100; i = i + 1) begin : gen_loop
			DFlipFlop D(tick, snake_body[12*i - 1:(i-1)*12], snake_body[12*(i+1)-1: 12*i], tail[i]);
		end
	endgenerate
	integer k;
	initial begin
		tail = 25000'b0;
		snake_length = 1;
		in = 12'b011001011001;
		snake_direction = 2'b00;
		lose = 0;
		counter = 0;
		tps = 32'd15;
		for (k = 0; k < 10; k = k + 1) begin
			snake_length = snake_length + 1;
			tail[snake_length - 1] = 1;
		end
	end
	
	always@( posedge tick ) begin
		
		if ( keyValue == 117 & snake_direction != 2'b11 ) begin // Change Up if not going Down.
			snake_direction = 2'b0;
		end else if ( keyValue == 116 & snake_direction != 2'b10 ) begin // Change Right if not going Left.
			snake_direction = 2'b01;
		end else if ( keyValue == 114 & snake_direction != 2'b00 ) begin // Change Down if not going Up.
			snake_direction = 2'b11;
		end else if ( keyValue == 107 & snake_direction != 2'b01 ) begin // Change Left if not going Right.
			snake_direction = 2'b10;
		end
		
		if (~lose& snake_direction == 2'sb0 ) begin // Up.
			new_head_x = in[11:6] + 0;
			new_head_y = in[5:0] - 1;
		end else if ( ~lose&snake_direction == 2'sb01 ) begin // Right.
			new_head_x = in[11:6] + 1;
			new_head_y = in[5:0] + 0;
		end else if ( ~lose&snake_direction == 2'sb11 ) begin // Down.
			new_head_x = in[11:6] + 0;
			new_head_y = in[5:0] + 1;
		end else if ( ~lose&snake_direction == 2'sb10 ) begin // Left.
			new_head_x = in[11:6] - 1;
			new_head_y = in[5:0] + 0;
		end
		

		if ((in[5:0] == 0 & snake_direction == 2'b00) | (in[5:0] == 63 & snake_direction == 2'b11) | (in[11:6] == 0 & snake_direction == 2'b10) | (in[11:6] == 63 & snake_direction == 2'b01)) begin
			lose = 1'b1;
		end
		// Head to Tail collision detection.
		for ( k = 1; k < 100; k = k + 1 ) begin
			if ((k < snake_length) & (new_head_x == snake_body[(k*12)+6 +: 6]) & (new_head_y == snake_body[(k*12) +: 6])) begin
				lose = 1'b1;
			end
		end
		
		if ((nom[5:0] == new_head_y) & (nom[11:6] == new_head_x)) begin
			tps = tps + 1;
			for (k  = 0; k < 4; k = k + 1) begin
				snake_length = snake_length + 1;
				tail[snake_length-1] = 1;
			end
			eat = 1'b1;
		end else begin
			eat = 1'b0;
		end
		in[11:6] = new_head_x;
		in[5:0] = new_head_y;
		if(lose) begin
			counter = counter + 1;
			if(counter == 100)begin 
				tail = 25000'b0;
				snake_length = 1;
				in = 12'b011001011001;
				snake_direction = 2'b00;
				lose = 0;
				counter = 0;
				tps = 32'd15;
				for (k = 0; k < 10; k = k + 1) begin
					snake_length = snake_length + 1;
					tail[snake_length - 1] = 1;
				end
			end
		
		end
	end
	
	rand_y ry(eat, nom[5:0]);
	rand_x rx(eat, nom[11:6]);
	assign LEDR = nom;
	assign LEDG = lose;
	
	VGAController(CLOCK_50, CLOCK_27, GPIO_0, GPIO_1, TD_RESET, VGA_CLK, VGA_HS, VGA_VS, VGA_BLANK, VGA_SYNC, VGA_R, VGA_G, VGA_B, snake_body, nom, snake_length, lose);
	keyboardController(CLOCK_50, PS2_DAT, PS2_CLK, GPIO_0, GPIO_1, keyValue);
	
	Tick(CLOCK_50, tick, tps);
		
endmodule

module Tick(Clk, tick, tps);
	output reg tick;
	input [31:0] tps; 
	input Clk;
	reg [26:0] clockcount;
	always@(posedge Clk)
	begin
		clockcount <= clockcount+1;
		if(clockcount >= 50000000/tps)
		begin
			tick  <= 1;
			clockcount <= 0;
		end
		else	
			tick <= 0;
	end
endmodule

module DFlipFlop(input CLK, input [11:0] D, output reg [11:0] Q, input enable);
	always@(posedge CLK) begin
		if(enable)
			Q = D;
		else	
			Q = 12'b0;
	end
endmodule
module render(input [9:0] x,
	input [9:0] y,
	output [9:0] red,
	output [9:0] green,
	output [9:0] blue,
	input [29999:0] snake_body,
	input [11:0] nom,
	input [31:0] snake_length,
	input lose);
	
	reg [2:0] idx;
	integer i;
	
	always @(x or y) begin
		//canvas
		integer found = 0;
	
		if (y >= 16 & y < 464 & x >= 96 & x < 544) begin
			found = 0;
			for (i = 0; i < 100; i = i + 1) begin
				if ((i < snake_length) & (x >= ((snake_body[(i*12)+6 +: 6]*7) + 96)) & (x < ((snake_body[(i*12)+6 +: 6]*7) + 7 + 96)) & (y >= ((snake_body[(i*12) +: 6]*7) +16)) & (y < ((snake_body[(i*12) +: 6]*7) + 7 + 16))) begin
						found = 1;
				end
			end
			if ((x >= ((nom[11:6]*7) + 96)) & (x < ((nom[11:6]*7) + 7 + 96)) & (y >= ((nom[5:0]*7) + 16)) & (y < ((nom[5:0]*7) + 7 + 16))) begin
				found = 2;
			end
			if (found == 1) begin
				idx = 3'b100;
			end else if (found == 2) begin
				idx = 3'b010;
			end else if (lose) begin
				idx = 3'b001;
			end else begin
				idx = 3'b111;
			end
		end else begin
			idx = 3'b0;
		end
	end
assign red = (idx[0] ? 10'h3ff : 10'h0);
assign green = (idx[1] ? 10'h3ff : 10'h0);
assign blue = (idx[2] ? 10'h3ff : 10'h0);

endmodule

module rand_x(input CLK, output [5:0] rand);	
	reg [15:0] Q;
	reg [31:0] count = 32'b0;
	reg temp;
	initial
		Q <= 16'b0111101001100101;
	always@(posedge CLK)
	begin
		temp <= Q[5]^(Q[3]^(Q[2]^Q[0]));
		Q <= Q >> 1;
		Q[15] <= temp;
	end
	assign rand = Q[15:10];
	
endmodule

module rand_y(input CLK, output [5:0] rand);	
	reg [15:0] Q;
	reg [31:0] count = 32'b0;
	reg temp;
	initial
		Q <= 16'b0010001110001001;
	always@(posedge CLK)
	begin
		temp <= Q[5]^(Q[3]^(Q[2]^Q[0]));
		Q <= Q >> 1;
		Q[15] <= temp;
	end
	assign rand = Q[15:10];
endmodule


module hex_7seg(hex_digit,seg);
input [3:0] hex_digit;
output [6:0] seg;
reg [6:0] seg;
// seg = {g,f,e,d,c,b,a};
// 0 is on and 1 is off

always @ (hex_digit)
case (hex_digit)
		4'h0: seg = 7'b1000000;
		4'h1: seg = 7'b1111001; 	// ---a----
		4'h2: seg = 7'b0100100; 	// |	  |
		4'h3: seg = 7'b0110000; 	// f	  b
		4'h4: seg = 7'b0011001; 	// |	  |
		4'h5: seg = 7'b0010010; 	// ---g----
		4'h6: seg = 7'b0000010; 	// |	  |
		4'h7: seg = 7'b1111000; 	// e	  c
		4'h8: seg = 7'b0000000; 	// |	  |
		4'h9: seg = 7'b0011000; 	// ---d----
		4'ha: seg = 7'b0001000;
		4'hb: seg = 7'b0000011;
		4'hc: seg = 7'b1000110;
		4'hd: seg = 7'b0100001;
		4'he: seg = 7'b0000110;
		4'hf: seg = 7'b0001110;
endcase
endmodule