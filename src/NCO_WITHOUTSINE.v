`default_nettype none

module tt_um_dds (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered, so you can ignore it
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

  // All output pins must be assigned. If not used, assign to 0.
  //assign uo_out  = 0;  // Example: ou_out is the sum of ui_in and uio_in
  assign uio_out = 0;
  assign uio_oe  = 0;
	assign uo_out[7:6]=uio_in[7:6];
  // List all unused inputs to prevent warnings
	wire _unused = &{,rst_n, 1'b0};
   DDS_top dds_nosine(
    .clock_in(clk),
    .enable_in(ena),
    .wavesel_in(ui_in[7:6]),
    .FTW_IN(uio_in[5:0]),   
    .amp_in(ui_in[5:0]),
    .finwave_out(uo_out[5:0])
    );
  endmodule
module DDS_top #(parameter width_top=6)(clock_in,enable_in,wavesel_in,FTW_IN,amp_in,finwave_out);  //
 input clock_in , enable_in ;
 input [width_top-1:0] FTW_IN;
	 input [5:0] amp_in;   // problem with amplitude
 input [1:0] wavesel_in;
 
	 output signed [5:0] finwave_out;
 
	 wire signed [6:0] amp_temp ; // to convert amp to signed 
	 wire signed [5:0] nco_wave;
	 wire signed [12:0] mult_result;   // xxxxx
 
 //wire signed [11:0] nco_wave_sign;
assign amp_temp = {1'b0,amp_in};

NCO #(.width(width_top)) nco1  
         (
          .clk_in(clock_in), .enable_in(enable_in),
		.FTW_in(FTW_IN), .wavesel_in(wavesel_in), 
		.wave_out(nco_wave)          
          );


assign mult_result = amp_temp * nco_wave;
assign finwave_out = mult_result [11:6] ;      //    nco_wave


endmodule 

module NCO #(parameter width = 6) (clk_in,enable_in,FTW_in,wavesel_in,wave_out);

input clk_in,enable_in;
input [width-1 :0] FTW_in;
input [1:0] wavesel_in;

	output reg signed [5:0] wave_out;

//wire sine_en;
wire updown_acc;
	wire [width-1 :0] phase_r;
	wire signed [5:0] square1,sawtooth1,triangular1 ;

accumulator i0  
       (	  .clk_in(clk_in), .enable_in(enable_in),
		  .updown_in(updown_acc), .freq_in(FTW_in),
		  .count_out(phase_r)
	 );
			  
always @(*)
begin			  
	case(wavesel_in) 
	2'b00: wave_out <= 0;
	2'b01: wave_out <= square1;
	2'b10: wave_out <= sawtooth1;
	2'b11: wave_out <= triangular1;
	default : wave_out <= 0;
	endcase 
end 

assign updown_acc = (wavesel_in == 2'b11) ? 1'b1 : 1'b0;
//assign sine_en = (wavesel_in == 2'b00) ? 1'b1 : 1'b0;

	assign sawtooth1 = phase_r[width-1 : width-6] - 32;
	assign triangular1 = phase_r[width-1 : width-6] - 32; //truncating phase accumulator bits to 12 
	assign square1 = (phase_r[5]==1) ? 6'b100000 : 6'b011111;

endmodule 

module accumulator #(parameter W = 6) (clk_in,enable_in,updown_in,freq_in,count_out);
input clk_in,enable_in,updown_in;
input [W-1 :0] freq_in;
output [W-1 :0] count_out; 

localparam [W-1 :0] max_count = 2**W -1 ;  // doubt with this assignment  12'b1111_1111_1111    

reg [W-1 :0] delta_inc ,delta_inc2,count_temp;
reg dir_temp;                    // to keep track of count direction

reg [W-1 :0] count_reg = 0;
reg dir_reg;                      

always @(*)
begin

delta_inc = freq_in;
delta_inc2 = 2*freq_in;  // for triangular wave calculation (multiply by 2 is just to inc slope)

if (updown_in == 0) begin   // accumulator value increments. this phase is used for wave other than triangular
    dir_temp = 0;
	 if (count_reg < (max_count - delta_inc)) 
	    count_temp = count_reg + delta_inc;  
	 else   
	    count_temp <= delta_inc - (max_count - count_reg); // overloading 
     
	end
else begin                     // accumulator values both increment and decrement. this phase is used for triangular wave
     
     if(dir_reg ==0) begin     // up
	     if(count_reg < (max_count - delta_inc2)) begin
		     count_temp = count_reg + delta_inc2;
			  dir_temp = 0; end 
		  else  begin
		     count_temp <= max_count - delta_inc2 - (max_count - count_reg); // overaloading
           dir_temp <= 1; end 
		end 
	   else   begin              // down
	      if (count_reg > delta_inc2) begin                                               
                    count_temp <= count_reg - delta_inc2;
                    dir_temp <= 1; end 
          else  begin                                                             
                    count_temp <= delta_inc2 - count_reg; // overloading 
                    dir_temp <= 0; end 
	   end 
	 
	 end 
end 


always @(posedge clk_in)
begin
  
  if (enable_in) begin
    count_reg <= count_temp;
	 dir_reg <= dir_temp ; end 
  else
    count_reg<=0;
end 

assign count_out = count_reg;

endmodule 
