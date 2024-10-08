with STM32.GPIO;             use STM32.GPIO;
with HAL;                    use HAL;
with STM32.Device;           use STM32.Device;
with STM32.Timers;           use STM32.Timers;
with Hardware_Configuration; use Hardware_Configuration;

package body Fans is

   procedure Init is
      procedure Init_Timer (Tim : in out Timer; Channel : Timer_Channel; Polarity : Timer_Output_Compare_Polarity) is
      begin
         Enable_Clock (Tim);
         Disable (Tim); --  The same timer may be used for multiple channels.
         if Advanced_Timer (Tim) then
            Enable_Main_Output (Tim);
         end if;
         Configure (This => Tim, Prescaler => 74, Period => 60_000); --  33.33 Hz
         Configure_Channel_Output
           (This => Tim, Channel => Channel, Mode => PWM1, State => Enable, Pulse => 0, Polarity => Polarity);
         Enable (Tim);
      end Init_Timer;
   begin
      for Fan in Fan_Name loop
         Init_Timer (Fan_Timers (Fan).all, Fan_Timer_Channels (Fan), Fan_Timer_Polarities (Fan));

         Configure_IO
           (Fan_GPIO_Points (Fan),
            (Mode           => Mode_AF,
             Resistors      => Floating,
             AF_Output_Type => Push_Pull,
             AF_Speed       => Speed_25MHz,
             AF             => Fan_GPIO_AFs (Fan)));
      end loop;
   end Init;

   procedure Set_PWM (Fan : Fan_Name; Scale : Fixed_Point_PWM_Scale) is
   begin
      Set_Compare_Value (Fan_Timers (Fan).all, Fan_Timer_Channels (Fan), UInt16 (Float (Scale) * 60_001.0));
   end Set_PWM;

end Fans;
