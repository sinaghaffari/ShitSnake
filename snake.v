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
  //hello world
);
	
	reg [9:0] xm = 10'd96;
	reg [9:0] ym = 10'd35;
	wire [9:0] x = xm;
	wire [9:0] y = ym;
	wire [7:0] keyValue;
	wire CLK = ~KEY[0];
	wire [29999:0] snake_wire = snake_body;
	reg [29999:0] snake_body;
	reg lose;
	assign LEDR = snake_body[11:0];
	/*always@(posedge CLK)begin
		case(keyValue)
			116: //right
				xm = xm + 9;
			114: //down
				ym = ym + 9;
			107: //left
				xm = xm - 9;
			117: //up
				ym = ym - 9;
		endcase
	end*/
	VGAController(CLOCK_50, CLOCK_27, GPIO_0, GPIO_1, TD_RESET, VGA_CLK, VGA_HS, VGA_VS, VGA_BLANK, VGA_SYNC, VGA_R, VGA_G, VGA_B, snake_wire[30000:0]);
	keyboardController(CLOCK_50, PS2_DAT, PS2_CLK, GPIO_0, GPIO_1, keyValue);
	
	
	
	//NOW WE SHALL BEGIn
	/* snake_length_bitstring has one bit for each of the segments of our snake. If the snake is n segments long, snake_length_bitstring will consist of (n - 1) one's, followed by (2499 - n) zeroes.
	 * This will be used to determine if the shift register should pass on the value of one segment to the next.
	 */
	reg [11:0] snake_length;
	reg [5:0] new_head_x; // Represents the current x coordinate of the head of the snake.
	reg [5:0] new_head_y; // Represents the current y coordinate of the head of the snake.
	reg [1:0] snake_direction; // The value of the FSM that determines the snake direction. Can be a value from 00 - 11.
	reg [1:0] dx; // Is the snake moving left or right?
	reg [1:0] dy; // Is the snake moving up or down?
	
	initial begin // Initialize everything to 0.
		snake_direction = 2'b0; // Initialize snake moving upwards.
		snake_length = 12'b000000000001; 
		snake_body = 30000'b0;
		lose = 1'b0;
		snake_body[11:0] =  12'b011001011001; // Set the current head of the snake to b the centre of the screen.
	end
	reg [16:0] i;
	always@( posedge CLK ) begin
	// Handle the last snake block.
		snake_body[29999:29988] = 12'b0; // No matter what, the tail of our shift register will drop it's current value.
	
	// Handle everything other than the last and first snake block.
		for (i = 2498; i > 0; i = i - 1) begin
			snake_body[i * 12 + 11:i * 12] = snake_body[i*12 - 1:i*12 - 12] & (snake_length < (i - 1));
		end

	// Handle the first snake block.
		if ( keyValue == 117 & ~snake_direction == 2'b11 ) begin // Change Up if not going Down..
			snake_direction = 2'b0;
		end else if ( keyValue == 116 & ~snake_direction == 2'b10) begin // Change Right if not going Left.
			snake_direction = 2'b01;
		end else if ( keyValue == 114 & ~snake_direction == 2'b00) begin // Change Down if not going Up.
			snake_direction = 2'b11;
		end else if ( keyValue == 107 & ~snake_direction == 2'b01) begin // Change Left if not going Right.
			snake_direction = 2'b10;
		end
		
		if (snake_direction == 2'sb0) begin // Up.
			new_head_x = snake_body[11:6] + 0;
			new_head_y = snake_body[5:0] - 1;
		end else if (snake_direction == 2'sb01) begin // Right.
			new_head_x = snake_body[11:6] + 1;
			new_head_y = snake_body[5:0] + 0;
		end else if (snake_direction == 2'sb11) begin // Down.
			new_head_x = snake_body[11:6] + 0;
			new_head_y = snake_body[5:0] + 1;
		end else if (snake_direction == 2'sb10) begin // Left.
			new_head_x = snake_body[11:6] - 1;
			new_head_y = snake_body[5:0] + 0;
		end

		// Check to see if you hit the borders.
		if (new_head_x < 0 | new_head_x >= 50 | new_head_y < 0 | new_head_y >= 50) begin
			lose = 1'b1;
		end

		// Check for snake on snake collision.
		// Having a dedicated loop for it may be a little bit inefficient. It would be nice to be able to combine it with the above loop.
		for (i = 1; i < snake_length; i = i + 1) begin
			if (new_head_x == snake_body[i*12+11:i*12 + 6] | new_head_y == snake_body[i*12+5:i*12]) begin
				lose = 1'b1;
			end
		end

		snake_body[11:6] = new_head_x;
		snake_body[5:0] = new_head_y;
	end	
endmodule


module render(input [9:0] x, input [9:0] y,
	output [9:0] red, 
	output [9:0] green, 
	output [9:0] blue, 
	input [30000:0] snake_body);
	reg [2:0] idx;
	
	always @(x or y)
	begin
	//canvas
	if (y > 15 & y < 465 & x > 95 & x < 545) begin
		//build snake
		//I'd make a for loop here, and check the snake length, and then render it
		if((x >= snake_body[5:0]*9 & x <= snake_body[5:0]*9 + 9 & y >= snake_body[11:6]*9 & y <= snake_body[11:6]*9 + 9))
			idx = 3'b101;
		else
			idx = 3'b111;
	end else begin
		idx = 3'b0;
	end
	end
assign red = (idx[0] ? 10'h3ff : 10'h0);
assign green = (idx[1] ? 10'h3ff : 10'h0);
assign blue = (idx[2] ? 10'h3ff : 10'h0);

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

	
