// drawer.v - a module to read two BRAMs in an alternating fashion (not at the same time),
//		and depending on the row and column that the VGA is currently displaying,
//		outputs a 64x64 cell grid, where each (4x4 pixel) cell is one bit in BRAM.
//		If the bit is a 0, the cell is gray, and if 1, the cell is pink.  A black 
//		square borders the pixel grid. Since each BRAM address holds 32 bits, and
//		there are 64x64 cells, addresses are designated as follows:
//
//		Left half of display: addr 0-255, Right half: addr 256-511 
// 
// Wesley Chavez
// 3/13/17
// 
//
//
//
//
///////////////////////////////////////////////////////////////////////////
	module drawer(clk,reset,video_on,bram_display,pixel_row,pixel_column,BRAM_PORTB_0_dout,BRAM_PORTB_1_dout,vga_red,vga_green,vga_blue,BRAM_PORTB_0_addr,BRAM_PORTB_0_en,BRAM_PORTB_1_addr,BRAM_PORTB_1_en);

	input clk,reset;

	// Inputs from DTG	
	input video_on;
	input [9:0] pixel_row;
	input [9:0] pixel_column;

	// Inputs from GPIO
	input bram_display;

	// Inputs from BRAM (data we read)
	input [31:0] BRAM_PORTB_0_dout;
	input [31:0] BRAM_PORTB_1_dout;
        
	// Outputs to VGA
 	output reg [3:0] vga_red;
	output reg [3:0] vga_green;
	output reg [3:0] vga_blue;

	// Outputs to BRAMs    
	output reg [31:0] BRAM_PORTB_0_addr;
	output BRAM_PORTB_0_en;

	output reg [31:0] BRAM_PORTB_1_addr;
	output BRAM_PORTB_1_en;



	// Alive is pink, dead is gray.
	parameter alive = 3864; //1111 0001 1000
	parameter dead = 2184;  //1000 1000 1000

	// To make cells bigger than 1x1 pixel don’t increase column as fast
	wire [7:0] pixel_column_8bit;	
	assign pixel_column_8bit = pixel_column[9:2];
    
	// bram_display tells us which BRAM to read from
	assign BRAM_PORTB_0_en = ~bram_display;
	assign BRAM_PORTB_1_en = bram_display;
    
	always @ (*) begin      
        
	// If displaying outside the border, send VGA the color black.
	if(pixel_row < 112 || pixel_row > 366 || pixel_column_8bit < 48 || pixel_column_8bit > 110) begin
            vga_red <= 0;
            vga_green <= 0;
            vga_blue <= 0;
            BRAM_PORTB_0_addr <= {22'b0, pixel_row - 112};
            BRAM_PORTB_1_addr <= {22'b0, pixel_row - 112};
        end

	// Inside grid border
        else begin
        
		// Read from BRAM 0
            if (bram_display == 0) begin

		// Read appropriate address based on if the column is on the left or right half
                if (pixel_column_8bit < 80) begin
                    BRAM_PORTB_0_addr <= {22'b0, pixel_row - 112};
                end
                else begin
                    BRAM_PORTB_0_addr <= {22'b0, pixel_row + 144};
                end

		// If the bit (based on the column) is zero, output gray
                if (BRAM_PORTB_0_dout[111-pixel_column_8bit] == 0) begin
                    vga_red <= dead[11:8];
                    vga_green <= dead[7:4];
                    vga_blue <= dead[3:0];
                end
		// Otherwise output pink
                else begin
                    vga_red <= alive[11:8];
                    vga_green <= alive[7:4];
                    vga_blue <= alive[3:0];
                end
            end

		// Read from BRAM 1, same logic
            else begin
                if (pixel_column_8bit < 80) begin
                    BRAM_PORTB_1_addr <= {22'b0, pixel_row - 112};
                end
                else begin
                    BRAM_PORTB_1_addr <= {22'b0, pixel_row + 144};
                end
                if (BRAM_PORTB_1_dout[111-pixel_column_8bit] == 0) begin
                    vga_red <= dead[11:8];
                    vga_green <= dead[7:4];
                    vga_blue <= dead[3:0];
                end
                else begin
                    vga_red <= alive[11:8];
                    vga_green <= alive[7:4];
                    vga_blue <= alive[3:0];
                end
            end
        end               
	end // always @
endmodule
