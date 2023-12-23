//ALARM CLOCK MODULE
module alarm_clock(input clk_2Hz, reset, time_set, alarm_set, sthrs1min0, run,
	activate_alarm, alarm_reset, output logic [7:0] sec, min, hrs, sec_alarm,
	min_alarm, hrs_alarm, output logic alarm);
logic clk_Hz_c, clk_Hz_a, clk_1Hz, reset_, run_c, run_a, sthrs1min0_, clk_2to1; //clock and alarm variables
fdivby2 #(1) f2(clk_2to1, reset_, clk_1Hz);
timer clockt(clk_Hz_c, reset_, run_c, time_set, sthrs1min0_, sec, min, hrs); //clock instantiation
timer alarmt(clk_Hz_a, reset_, run_a, alarm_set, sthrs1min0_, sec_alarm, min_alarm, hrs_alarm); //alarm instantiation
always_comb begin
	clk_2to1 = clk_2Hz;
	clk_Hz_c = clk_1Hz;
	clk_Hz_a = 1'b0;
	reset_ = reset;
	run_c = run; 
	run_a = 1'b0; //run for alarm is always ZERO
	if(alarm_set == 1'b1) begin //if alarm is on stop clock
		clk_Hz_a = clk_2Hz;
		clk_Hz_c = 1'b0;
		run_c = 1'b0;
	end
	else if(time_set == 1'b1)begin //if clock is on stop alarm
		clk_Hz_a = 1'b0;	
		clk_Hz_c = clk_2Hz;
		run_c = 1'b0;
	end
	sthrs1min0_ = sthrs1min0;
end
D_ff D1((min==min_alarm)&&(hrs==hrs_alarm), reset|alarm_reset, activate_alarm, 1'b1, alarm); //D flip flop to handle the alarm
endmodule 

module timer(input clk_Hz, reset, run, set, hrs1_min0, output [7:0] Seconds, Minutes, Hours);
logic clk_sec, clk_min, clk_hr, enable_Sec, enable_Min, enable_Hrs, Min_in, Hr_in, Days_in; //variables for sec, min, hr clock instantiation
logic [7:0] fiftynine, twentythree;
assign fiftynine = 8'd59;
assign twentythree = 8'd23;
clocktime secclock(clk_sec, enable_Sec, reset, fiftynine, Seconds, Min_in);//1hz
clocktime minclock(clk_min, enable_Min, reset, fiftynine, Minutes, Hr_in);//2hz
clocktime hrclock(clk_hr, enable_Hrs, reset, twentythree, Hours, Days_in);
always_comb begin
	clk_sec = clk_Hz;
	clk_min = Min_in;
	clk_hr = Hr_in;
	enable_Sec = 1'b0; enable_Min=1'b0; enable_Hrs=1'b0;
	if(run) begin // run clock time if run = 1
		clk_sec = clk_Hz;
		clk_min = Min_in;
		clk_hr = Hr_in;
		enable_Sec = 1'b1; enable_Min=1'b1; enable_Hrs=1'b1;// all three set to run
	end
	else if(set) begin //if set is on run hours or minutes
		if(hrs1_min0) begin
			clk_sec = 1'b0; enable_Sec = 1'b0; enable_Min=1'b0; enable_Hrs=1'b1; //hours set to run
			clk_hr = clk_Hz;
			clk_min = 1'b0; //dont run minutes
			end
		else begin
			clk_sec = 1'b0; enable_Sec = 1'b0; enable_Min=1'b1; enable_Hrs=1'b0; //minutes set to run
			clk_hr = 1'b0; //dont set hours
			clk_min = clk_Hz;
			end
		end
	end
endmodule




module clocktime(input clk, enable, reset, input [7:0] Maxval, output logic [7:0] Count, output logic clkout); //counter for clock
localparam Zero = 8'd0, One = 8'd1, zero = 1'b0, one = 1'b1; //variables that represent numbers
always_ff @ (posedge clk or posedge reset) begin
	if(reset) begin
		Count <= Zero;
		clkout <= zero;
	end
	else if(enable)
		if(Count<Maxval) begin
			Count <= Count + One;
			clkout <= zero;
		end
		else begin
			Count <= Zero;
			clkout <= one;
		end
	end
endmodule

module D_ff (input clk, Reset, Enable, D, output reg Q); //D flip flop to handle the alarm
always_ff@ (posedge clk or posedge Reset)
	if(Reset)
		Q <= 1'b0;
	else if(Enable)
		Q <= D;
endmodule

module fdivby2 #(parameter size = 7) (input clk, reset, output logic clkout); //frequency divider
always_ff @ (posedge clk or posedge reset)
	if(reset)
		clkout <= 1'b0;
	else
		clkout <= ~clkout;
endmodule

module pmcntr #(parameter size = 5) (input clk, reset, input [size-1:0] count_max, //counter to make 50M hz equal 2 hz
	output logic [size-1:0] count, output logic clkout);
always_ff @ (posedge clk or posedge reset) begin
	if(reset) begin
		count <= {size{1'b0}};
		clkout <= 1'b0;
	end
	else if (count < count_max) begin
		count <= count + {{(size-1){1'b0}}, 1'b1};
	end
	else begin
		count <= {size{1'b0}};
		clkout <= ~clkout;
	end
end
endmodule


`timescale 1ns/1ps
module alarm_clock_tb(); //ALARM CLOCK TEST BENCH
logic clk_2Hz, reset, sethrs1min0,activatealarm, alarmreset, runset, time_set, alarm_set, alarm;
logic [7:0] sec, min, hrs, sec_alarm, min_alarm, hrs_alarm;
alarm_clock a1(clk_2Hz, reset, time_set, alarm_set, sethrs1min0, runset, activatealarm, alarmreset, sec, min, hrs,sec_alarm, min_alarm, hrs_alarm, alarm);
initial begin
	reset = 1; clk_2Hz = 0; #10;
	reset = 0; clk_2Hz = 0; #10;
	time_set = 0; alarm_set = 0; sethrs1min0 = 0;  activatealarm = 0; runset = 0; alarmreset = 1;
	alarm_set = 1;
	sethrs1min0 = 0;
	repeat(22) begin //set time to 22 minutes
		clk_2Hz = 1; #10 clk_2Hz = 0; #10;
	end
	sethrs1min0 = 1;
	repeat(7) begin //set time to 7  hours
		clk_2Hz = 1; #10 clk_2Hz = 0; #10;
	end
	alarm_set = 0; time_set = 1;
	sethrs1min0 = 0;
	repeat(21) begin //set time to 21 minutes
		clk_2Hz = 1; #10 clk_2Hz = 0; #10;
	end
	sethrs1min0 = 1;
	repeat(7) begin //set time to 5 hours
		clk_2Hz = 1; #10 clk_2Hz = 0; #10;
	end
	time_set = 0; alarmreset = 0;
	clk_2Hz = 1; #10; 
	clk_2Hz = 0; #10;
	runset = 1; activatealarm = 1;
	repeat(120) begin
		clk_2Hz = 1; #10;
		clk_2Hz = 0; #10;
	end
	alarmreset = 1; clk_2Hz = 1; #10;
	clk_2Hz = 0; #10;
	clk_2Hz = 1; #10;
	clk_2Hz = 0; #10;
	
	//turn on alarm and run clock
end
endmodule

module find_msb_lsb(input [7:0] T, output [7:0] msb, lsb); //MSB LSB MODULE TO HANDLE DISPLAYS
assign msb = T/8'd10;
assign lsb = T - msb*8'd10;
endmodule

// ALARM CLOCK PHYSICAL VALIDATION MODULE
module alarm_clock_pv(input clk, SW5, SW4,SW3,SW2,SW1,SW0, KEY1,KEY0,
	output logic [6:0] SEC_LSD, SEC_MSD, MIN_LSD,
	MIN_MSD, HR_LSD, HR_MSD,
	output logic LED7, LED5, LED4,LED3,LED2,LED1,LED0);
logic s0, s1, s2, s3,s4, s5, clk_, clk_2Hz, clk_2Hz_; //switches and clock
logic [7:0] sec, min, hrs,sec_alarm, min_alarm, hrs_alarm; //time variables for instance to find_msb_lsb connection
logic [7:0] sec_c, min_c, hrs_c, sec_a, min_a, hrs_a;  //time variables for instance to find_msb_lsb connection
logic [7:0] sec_c_msb, sec_c_lsb, min_c_msb, min_c_lsb, hrs_c_msb, hrs_c_lsb;  //time variables for find_msb_lsb to display ASCII connection
logic [7:0] sec_a_msb, sec_a_lsb, min_a_msb, min_a_lsb, hrs_a_msb, hrs_a_lsb; //time variables for find_msb_lsb to display ASCII connection
logic alarm, button0; // alarm and key0 variables
logic [22:0] count; //%50 counter variable
logic [7:0] display [5:0]; //decimal display
find_msb_lsb sec_cl(sec_c, sec_c_msb, sec_c_lsb); //seconds clock
find_msb_lsb min_cl(min_c, min_c_msb, min_c_lsb); //minutes clock
find_msb_lsb hrs_cl(hrs_c, hrs_c_msb, hrs_c_lsb); //hours clock
find_msb_lsb sec_al(sec_a, sec_a_msb, sec_a_lsb); //seconds alarm
find_msb_lsb min_al(min_a, min_a_msb, min_a_lsb); //minutes alarm
find_msb_lsb hrs_al(hrs_a, hrs_a_msb, hrs_a_lsb); //hours alarm
pmcntr #(23) p1 (clk_, s1, 23'd5000000, count, clk_2Hz_);
alarm_clock a1(clk_2Hz, s0, s2, s1, s3, s5, s4, button0, sec, min, hrs,sec_alarm, min_alarm, hrs_alarm, alarm); //alarm clock instanction
ASCII_D d5(display[5], HR_MSD); 
ASCII_D d4(display[4], HR_LSD);
ASCII_D d3(display[3], MIN_MSD);
ASCII_D d2(display[2], MIN_LSD);
ASCII_D d1(display[1], SEC_MSD);
ASCII_D d0(display[0], SEC_LSD);
//always_comb begin
always_ff @(posedge clk_2Hz) begin
	display[5] = hrs_c_msb; //display for clock
	display[4] = hrs_c_lsb;
	display[3] = min_c_msb;
	display[2] = min_c_lsb;
	display[1] = sec_c_msb;
	display[0] = sec_c_lsb;
	if(SW1) begin
		display[5] = hrs_a_msb; // display for alarm that doesn't count up
		display[4] = hrs_a_lsb;
		display[3] = min_a_msb;
		display[2] = min_a_lsb;
		display[1] = 8'b0;
		display[0] = 8'b0;
	end
	else begin
		display[5] = hrs_c_msb;
		display[4] = hrs_c_lsb;
		display[3] = min_c_msb;
		display[2] = min_c_lsb;
		display[1] = sec_c_msb;
		display[0] = sec_c_lsb;
	end
	LED7 = 1'b0;
	if(alarm == 1'b1)
		LED7 = 1'b1;
end

always_comb begin
	clk_ = clk; // variable connections
	clk_2Hz = clk_2Hz_;
	s0 = SW0;
	s1 = 1'b0;
	s2 = 1'b0;
	button0 = 1'b0;
	s3 = SW3;
	s4 = SW4;
	s5 = SW5;
	sec_c = sec;
	min_c = min;
	hrs_c = hrs;
	sec_a = sec_alarm;
	min_a = min_alarm;
	hrs_a = hrs_alarm;
	if (KEY0 == 1'b0) begin
		button0 = 1'b1;
	end
	if((KEY1==1'b0) & (SW1 == 1'b1)) begin // if switch one is up and key is pressed, the alarm counts DOESNT WORK
		s1 = 1'b1;
	end
	if((KEY1==1'b0) & (SW2 == 1'b1)) begin  // if switch two is up and key is pressed, the alarm counts  works
		s2 = 1'b1;
	end
	if((SW1 == 1'b1) | (SW2 == 1'b1)) begin //if a switch is up, dont count
		s5 = 1'b0;
	end
end
always_comb begin
	LED0 = 1'b0; // SWITCHES to LED connection
	LED1 = 1'b0;
	LED2 = 1'b0;
	LED3 = 1'b0;
	LED4 = 1'b0;
	LED5 = 1'b0;
	if(SW0)
		LED0 = 1'b1;
	else
		LED0 = 1'b0;
	if(SW1)
		LED1 = 1'b1;
	else
		LED1 = 1'b0;
	if(SW2)
		LED2 = 1'b1;
	else
		LED2 = 1'b0;
	if(SW3)
		LED3 = 1'b1;
	else
		LED3 = 1'b0;
	if(SW4)
		LED4 = 1'b1;
	else
		LED4 = 1'b0;
	if(SW5)
		LED5 = 1'b1;
	else
		LED5 = 1'b0;

end
endmodule

module ASCII_D(input [7:0] Decimal, output reg [6:0] HexSeg);
	always @ (*) begin
		HexSeg = 7'd0;
		case(Decimal)
			8'h0 : HexSeg = 7'b1000000;
			8'h1 : HexSeg = 7'b1111001;
			8'h2 : HexSeg = 7'b0100100; // 2
			8'h3 : HexSeg = 7'b0110000; // 3
			8'h4 : HexSeg = 7'b0011001; // 4
			8'h5 : HexSeg = 7'b0010010; // 5
			8'h6 : HexSeg = 7'b0000010; // 6
			8'h7 : HexSeg = 7'b1111000; // 7
			8'h8 : HexSeg = 7'b0000000; // 8
			8'h9 : HexSeg = 7'b0010000; // 9
		endcase
	end
endmodule
