`default_nettype none

module tt_um_adpll (
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
  // List all unused inputs to prevent warnings
	wire _unused = &{ena,uio_in,1'b0};
   adpll adp(
    .vco_clk(clk),
    .clk2_5k(ui_in[0]),
    .tdc_reset(rst_n),   
    .out(uo_out[5]),
    .vco_in(uo_out[3:0]),
    .desired_out(uo_out[4])
    );
  endmodule
module adpll(vco_clk, clk2_5k, tdc_reset, out, vco_in,desired_out);
	input tdc_reset;
	input vco_clk,clk2_5k;// clk2_5k;
	output out;// clk2_5k;
	output desired_out;
	output [3:0] vco_in;
	wire [15:0] s;
	wire [15:0] t;
	wire freq_div_out;
	wire [3:0] encoder_out;
	
	// FLASH TDC with resolution = 24 us 
	tdc_in m1(tdc_reset, vco_clk, freq_div_out, s);
	tdc_ref m2(clk2_5k, s, t);
	
	// Produces VCO_IN based on the digital code obtained from TDC
	encoder m3(t, encoder_out);
	dlf m4(encoder_out, vco_in, clk2_5k);
	
	// Works on 50 MHz and provides the 16 frequencies from 2 KHz to 17 KHz
	vco m5(vco_clk, vco_in, out, desired_out);
	
	// Divide by 4 frequency divider
	freq_div m6(out, freq_div_out); 
endmodule

module tdc_in(tdc_reset, vco_clk, clk, out);
	input tdc_reset, vco_clk, clk;
	output reg [15:0] out=0;
//	integer i = 0; 
	wire en = 1;
	wire [11:0] squ_out;
	wire wave_out;
	wire ref24;
	

	waveform_gen dut (
    .clk(vco_clk),
    .reset(tdc_reset),
    .en(en),
    .phase_inc(32'h00038FCE2),//369D03), // to generate 24us signal which is the resolution of our TDC,
    .squ_out(squ_out),
	 .vco_out(ref24),
	 .desired_freq_sig(wave_out)
);
	
	always@(posedge ref24)
	begin
		out[0] <= clk;
		out[1] <= out[0];
		out[2] <= out[1];
		out[3] <= out[2];
		out[4] <= out[3];
		out[5] <= out[4];
		out[6] <= out[5];
		out[7] <= out[6];
		out[8] <= out[7];
		out[9] <= out[8];
		out[10] <= out[9];
		out[11] <= out[10];
		out[12] <= out[11];
		out[13] <= out[12];
		out[14] <= out[13];
		out[15] <= out[14];
	end
//	
endmodule

module tdc_ref(clk, in, out);
	input clk;
	input [15:0] in;
	output reg [15:0] out=0;
	integer i = 0;
	always@(posedge clk)
	begin
		for(i=0; i<16; i = i + 1)
		begin
			out[i] <= in[i];
		end
	end
endmodule

module encoder (in, out);
	input [15:0] in;
	output reg [3:0] out = 0;    
	always@(*)
	begin  
		if((16'hffff & in) == 16'hffff) out = 4'b0111;
		else if((16'h7fff & in) == 16'h7fff) out = 4'b0111;
		else if((16'h3fff & in) == 16'h3fff) out = 4'b0110;
		else if((16'h1fff & in) == 16'h1fff) out = 4'b0101;
		else if((16'h0fff & in) == 16'h0fff) out = 4'b0100;
		else if((16'h07ff & in) == 16'h07ff) out = 4'b0011;
		else if((16'h03ff & in) == 16'h03ff) out = 4'b0010;
		else if((16'h01ff & in) == 16'h01ff) out = 4'b0001;
		else if((16'h00ff & in) == 16'h00ff) out = 4'b1111;
		else if((16'h007f & in) == 16'h007f) out = 4'b1110;
		else if((16'h003f & in) == 16'h003f) out = 4'b1101;
		else if((16'h001f & in) == 16'h001f) out = 4'b1100;
		else if((16'h000f & in) == 16'h000f) out = 4'b1011;
		else if((16'h0007 & in) == 16'h0007) out = 4'b1010;
		else if((16'h0003 & in) == 16'h0003) out = 4'b1001;
		else if((16'h0001 & in) == 16'h0001) out = 4'b1000;
		else out = 4'b0000;
	end 
endmodule

module dlf (in, out, clk);
	input clk;
	input signed [3:0] in;
	output [3:0] out;
//	wire signed [3:0] acc;   ///original filter
//	reg signed [3:0] latch = 0; 
//	parameter a = 1;// b = 1;
//	assign acc = latch + (in >>> 2); //in/4
//	//assign out = acc + (in * a);
//	assign out = acc + in; //same as a=1 ?
//	always@(posedge clk)
//	begin
//		latch <= acc;
//	end


reg signed [3:0] latch1 = 0; // moving summation filter
reg signed [3:0] latch2 = 0;
reg signed [3:0] latch3 = 0;
wire signed [3:0] acc;

	always@(posedge clk)
	begin
		latch1 <= in;
	end
	
	always@(posedge clk)
	begin
		latch2 <= latch1;
	end	
	
	always@(posedge clk)
	begin
		latch3 <= latch2;
	end
	
	assign acc = latch1 + latch2 + latch3 +in;
	
	assign out= (acc>>0);

//	reg signed [3:0] latch1 = 0; // IIR filter
//reg signed [3:0] latch2 = 0;
//wire signed [3:0] acc;
//wire signed [3:0] shifter;
//	
//	always@(posedge clk)
//	begin
//		latch1 <= in;
//	end
//	
//	assign acc = latch1 + in;
//	
//	always@(posedge clk)
//	begin
//		latch2 <= out;
//	end	
//	
//	assign shifter=(latch2>>2);
//	
//	assign out=(shifter*4'd3)+acc;
endmodule 

module vco (vco_clk, in, out, desired_out);
   input vco_clk;
//	input reset;
	input [3:0] in;
	output wire out;
	output wire desired_out;
	
	reg [31:0] phase_inc;
	wire en=1;
	reg reset;
	reg [3:0]in_reg;
	wire got_it;
	wire [11:0] sin_out;
	wire [11:0] cos_out;
	wire [11:0] squ_out;
waveform_gen dut (
    .clk(vco_clk),
    .reset(reset),
    .en(en),
    .phase_inc(phase_inc),
    .sin_out(sin_out),
	 .cos_out(cos_out),
    .squ_out(squ_out),
	 .vco_out(out),
	 .desired_freq_sig(desired_out)
);

always@(posedge vco_clk)
	begin
		in_reg <= in;
		if((in_reg == in))
			reset <= 1;
		else
			reset <= 0;
	end

always@(*)
	begin
	//with VCO CLOCK = 50 MHz

		case (in)
			4'h0 :
			begin
				phase_inc = 32'h00029F16;//2 KHz 
			end
			4'h1 :
			begin
				phase_inc = 32'h0003EEA2;//3 KHz
			end  
			4'h2 :
			begin
				phase_inc = 32'h00053E2D;//4 KHz
			end  
			4'h3 :
			begin
				phase_inc = 32'h00068DB8;//5 KHz
			end 
			4'h4 :
			begin
				phase_inc = 32'h0007DD44;//6 KHz
			end
			4'h5 :
			begin
				phase_inc = 32'h00092CCF;//7 KHz
			end  
			4'h6 :
			begin
				phase_inc = 32'h000A7C5A;//8 KHz
			end  
			4'h7 :
			begin
				phase_inc = 32'h000BCBE6;//9 KHz
			end
			4'h8 :
			begin
				phase_inc = 32'h000D1B71; //10 KHz center
			end
			4'h9 :
			begin
				phase_inc = 32'h000E6AFC;//11 KHz
			end  
			4'ha :
			begin
				phase_inc = 32'h000FBA88;//12 KHz
			end  
			4'hb :
			begin
				phase_inc = 32'h00110A13;//13 KHz
			end
			4'hc :
			begin
				phase_inc = 32'h0012599E;//14 KHz
			end
			4'hd :
			begin
				phase_inc = 32'h0013A92A;//15 KHz
			end  
			4'he :
			begin
				phase_inc = 32'h0014F8B5;//16 KHz
			end  
			4'hf :
			begin
				phase_inc = 32'h00164840;//17 KHz
			end    
		endcase

	end

endmodule

module freq_div (clk, clk_out);
	input clk;
	output clk_out;
	reg [1:0] r_reg =2'd 0;
	wire [1:0] r_nxt;
	reg clk_track = 0; 
	always @(posedge clk) 
	begin
		if (r_nxt == 2'b10)
		begin
			r_reg <= 2'd0;
			clk_track <= ~clk_track;
		end
		else 
			r_reg <= r_nxt;
		end
	assign r_nxt = r_reg + 2'd1;   	      
	assign clk_out = clk_track;
endmodule


module waveform_gen (
    input wire clk,
	input wire reset,
    input wire en,
    input wire [31:0] phase_inc,
    output wire [11:0] squ_out,
	 output vco_out,
	 output desired_freq_sig);

reg [31:0] phase_acc;
wire read;
// Phase accumulator logic
always @(posedge clk ) begin //or negedge reset
    if (~reset) begin
        phase_acc <= 32'b0;
    end else if (en && ~read)
				begin
					phase_acc <= phase_acc + phase_inc;
				end
				else if(en&&read)
				begin
					phase_acc <= phase_acc + 32'h000D1B71;

				end
end


// Square wave output
assign squ_out = (lut_addr_reg[11]) ? 12'b011111111111 : 12'b100000000000;
assign vco_out = squ_out[11];

assign desired_freq_sig = read ? squ_out[11] : 1'b0;


endmodule
