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

   Server_Communication.Init;

   begin
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
   exception
      when E : Server_Communication.Timeout_Error =>
         Heaters.Make_Safe;

         Server_Communication.Transmit_String_Line (Ada.Exceptions.Exception_Message (E));
         Server_Communication.Transmit_String_Line ("Restarting.");
         Server_Communication.Transmit_Fatal_Exception_Mark;
      when E : others =>
         Heaters.Make_Safe;

         --  To view tracebacks:
         --  addr2line -e ./bin/prunt_board_1_firmware.elf -afp --demangle=gnat <address list here>

         --  Repeat for around 50 seconds before a reset.
         for I in reverse 0 .. 10 loop
            Server_Communication.Transmit_String_Line ("");
            Server_Communication.Transmit_String_Line ("Fatal exception on MCU:");
            Server_Communication.Transmit_String_Line (Ada.Exceptions.Exception_Information (E));
            Server_Communication.Transmit_String_Line ("Compilation date: " & GNAT.Source_Info.Compilation_ISO_Date);
            Server_Communication.Transmit_String_Line ("Compilation time: " & GNAT.Source_Info.Compilation_Time);
            Server_Communication.Transmit_String_Line
              ("Please note that this message will repeat " & I'Image &
               " more times at 5 second intervals before the board restarts.");
            Server_Communication.Transmit_Fatal_Exception_Mark;
            delay until Clock + (Seconds (5));
         end loop;
         Server_Communication.Transmit_String_Line ("Restarting.");
         Server_Communication.Transmit_Fatal_Exception_Mark;
         System.Machine_Reset.Stop;
   end;

end Prunt_Board_1_Firmware;
