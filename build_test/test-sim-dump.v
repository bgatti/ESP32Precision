// Custom testbench
`timescale 1ns / 1ps

module StateMachine_tb();
	parameter OSC_frequency = 50000000;
	parameter OSC_period = (0.5/OSC_frequency)*10E8;
    
	reg clk = 0;
    reg datapin = 0;
    reg accelpin = 0;
    reg flightpin = 0;
    wire terminatepin;

    // Instantiate the state machine module
    StateMachine u1 (
        .clk(clk),
        .datapin(datapin),
        .accelpin(accelpin),
        .flightpin(flightpin),
        .terminatepin(terminatepin)
    );

always 
  #OSC_period  clk = ! clk;    //create clk 50 MHz
    
    initial begin
      $dumpfile ("MAIN_testbench.vcd");
	  $dumpvars (0, StateMachine_tb);

        #25 accelpin = 1; flightpin = 1; // these pins pullup

        // Stimulate the inputs
        #100 datapin = 1;  // datapin goes high, powertime should reset and start counting
        #100 datapin = 0;  // datapin goes low, programtime should get the value of powertime 
        #100 datapin = 1;  // datapin goes high, powertime should reset and start counting
        #100 datapin = 0;  // datapin goes low, programtime should get the value of powertime 
        #100 accelpin = 0; // accelpin goes low, state should change to accel
        #100 flightpin = 0; // flightpin goes low, state should change to flight
        #2500 $finish;  //flighttime should reach programtime, state should change to terminate
    end


endmodule
