with STM32.Device;  use STM32.Device;
with STM32.GPIO;    use STM32.GPIO;
with Server_Communication;
with Step_Generator;
with Steppers;
with High_Power_Switch;
with Input_Switches;
with Thermistors;
with Heaters;
with Fans;
with Ada.Exceptions;
with Ada.Real_Time; use Ada.Real_Time;
with GNAT.Source_Info;
with System.Machine_Reset;

with Last_Chance_Handler;
pragma Unreferenced (Last_Chance_Handler);
--  The "last chance handler" is the user-defined routine that is called when
--  an exception is propagated. We need it in the executable, therefore it
--  must be somewhere in the closure of the context clauses.

procedure Prunt_Board_1_Firmware is
begin
   Enable_Clock (GPIO_A);
   Enable_Clock (GPIO_B);
   Enable_Clock (GPIO_C);
   Enable_Clock (GPIO_D);
   Enable_Clock (GPIO_F);

   --  Unused floating pins and BOOT0.
   Configure_IO (Points => (PD2, PA2, PA6, PB0, PB1, PB4, PB8), Config => (Mode => Mode_In, Resistors => Pull_Down));

   --  Always start server communication first so exceptions can be reported.
   Server_Communication.Init;

   Fans.Init;
   Input_Switches.Init;
   Steppers.Init;
   Step_Generator.Init;
   Heaters.Init;

   delay until Clock + Seconds (1); --  Ensure voltages have time to come up before ADC calibration.

   Thermistors.Init;
   High_Power_Switch.Init;

   High_Power_Switch.Wait_For_Power_Good;

   Server_Communication.Run;

end Prunt_Board_1_Firmware;
