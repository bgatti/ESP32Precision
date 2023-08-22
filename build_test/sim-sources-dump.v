`timescale 1ns / 1ps

(* top *) module StateMachine #(
/* --------------------------- Set hardware parameter --------------------------- */
	parameter OSC_frequency = 50000000,
	parameter distance_between_sensors = 0.3
	)(
    (* iopad_external_pin, clkbuf_inhibit *)		input clk,
    (* iopad_external_pin *)						input datapin,
    (* iopad_external_pin *)						input accelpin,
    (* iopad_external_pin *)						input flightpin,
    (* iopad_external_pin *)						output terminatepin,
    
  	// OSC config outputs
	(* iopad_external_pin *) output osc_en,
  	(* iopad_external_pin *) output osc_mode
    
);

   	reg [31:0] powertime = 0, acceltime = 0, flighttime = 0;
    reg [31:0] programtime = 25000; // 1000ms in resolution/4
    reg [2:0] state = 0; // states: boot(0), transfer(1), accel(2), flight(3), terminate(4) (000,001,010,011,100)
    reg terminatereg = 0;

	assign osc_en = 1'b1;
	assign osc_mode = 1'b1;
	assign terminatepin = terminatereg;
	
	always @(posedge datapin )
		powertime <= 0;
		
	always @(negedge datapin )
		programtime <= powertime;
		
	
    always @(posedge clk ) begin
            case (state)
                0: // boot state
                    if (datapin & accelpin & flightpin) 
                        state <= 1; // transition to transfer state
                1: // transfer state
                    begin
                        powertime <= powertime + 6;
                        if (!accelpin)
                            state <= 2; // transition to accel state
                    end
                2: // accel state
                    begin
                        acceltime <= acceltime + 1;
                        if (acceltime > 1000 || !flightpin) 
                            state <= 3; // transition to flight state
                            //programtime = programtime * (acceltime/expectedtime)
                    end
                3: // flight state
                    begin
                        flighttime <= flighttime + 1;
                        if (flighttime >= programtime)
                            state <= 4; // transition to terminate state
                    end
                4: // terminate state
                    begin
                        // additional actions for the terminate state could be placed here
						terminatereg = 1'b1;
                    end
                default: state <= 0; // boot state
            endcase
        end
endmodule
