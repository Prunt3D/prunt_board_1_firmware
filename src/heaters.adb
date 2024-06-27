with STM32.GPIO;   use STM32.GPIO;
with HAL;          use HAL;
with STM32.Device; use STM32.Device;
with STM32.Timers; use STM32.Timers;
with STM32.IWDG;
with Server_Communication;

package body Heaters is

   procedure Init is
      procedure Init_Timer (Tim : in out Timer; Channel : Timer_Channel; Polarity : Timer_Output_Compare_Polarity) is
      begin
         Enable_Clock (Tim);
         Configure (This => Tim, Prescaler => 2, Period => 50_000); --  1kHz
         Configure_Channel_Output
           (This => Tim, Channel => Channel, Mode => PWM1, State => Enable, Pulse => 0, Polarity => Polarity);
         Enable (Tim);
      end Init_Timer;
   begin
      for Heater in Heater_Name loop
         Init_Timer (Heater_Timers (Heater).all, Heater_Timer_Channels (Heater), Heater_Timer_Polarities (Heater));

         Configure_IO
           (Heater_GPIO_Points (Heater),
            (Mode           => Mode_AF,
             Resistors      => Floating,
             AF_Output_Type => Push_Pull,
             AF_Speed       => Speed_25MHz,
             AF             => Heater_GPIO_AFs (Heater)));
      end loop;
   end Init;

   procedure Make_Safe is
   begin
      for Heater in Heater_Name loop
         Configure_IO (Heater_GPIO_Points (Heater), (Mode => Mode_In, Resistors => Pull_Down));
      end loop;
   end Make_Safe;

   procedure Setup (Heater : Heater_Name; Parameters : Heater_Parameters) is
      Ctx : Context renames Contexts (Heater);
   begin
      pragma Assert (not Global_Update_Blocker);
      Global_Update_Blocker := True;

      Set_PWM (Heater, 0.0);

      case (Parameters.Kind) is
         when Disabled_Kind =>
            Ctx :=
              (Kind                 => Disabled_Kind,
               Setpoint             => 0.0,
               Max_Cumulative_Error => Celcius (Parameters.Max_Cumulative_Error),
               Check_Gain_Time      => To_Time_Span (Duration (Parameters.Check_Gain_Time)),
               Check_Minimum_Gain   => Celcius (Parameters.Check_Minimum_Gain),
               Hysteresis           => Celcius (Parameters.Hysteresis),
               Approaching_Setpoint => False,
               Starting_Approach    => False,
               Cumulative_Error     => 0.0,
               Last_Setpoint        => 0.0,
               Check_Goal_Temp      => 0.0,
               Check_Goal_Time      => <>);
         when Bang_Bang_Kind =>
            Ctx :=
              (Kind                 => Bang_Bang_Kind,
               Setpoint             => Contexts (Heater).Setpoint,
               Max_Cumulative_Error => Celcius (Parameters.Max_Cumulative_Error),
               Check_Gain_Time      => To_Time_Span (Duration (Parameters.Check_Gain_Time)),
               Check_Minimum_Gain   => Celcius (Parameters.Check_Minimum_Gain),
               Hysteresis           => Celcius (Parameters.Hysteresis),
               Approaching_Setpoint => False,
               Starting_Approach    => False,
               Cumulative_Error     => 0.0,
               Last_Setpoint        => 0.0,
               Check_Goal_Temp      => 0.0,
               Check_Goal_Time      => <>);
         when PID_Kind =>
            Ctx :=
              (Kind                        => PID_Kind,
               Setpoint                    => Ctx.Setpoint,
               Max_Cumulative_Error        => Celcius (Parameters.Max_Cumulative_Error),
               Check_Gain_Time             => To_Time_Span (Duration (Parameters.Check_Gain_Time)),
               Check_Minimum_Gain          => Celcius (Parameters.Check_Minimum_Gain),
               Hysteresis                  => Celcius (Parameters.Hysteresis),
               Approaching_Setpoint        => False,
               Starting_Approach           => False,
               Cumulative_Error            => 0.0,
               Last_Setpoint               => 0.0,
               Check_Goal_Temp             => 0.0,
               Check_Goal_Time             => <>,
               Proportional_Scale          => Float (Parameters.Proportional_Scale),
               Integral_Scale              => Float (Parameters.Integral_Scale),
               Derivative_Scale            => Float (Parameters.Derivative_Scale),
               Proportional_On_Measurement => Boolean (Parameters.Proportional_On_Measurement),
               Output_Sum                  => (if Ctx.Kind = PID_Kind then Ctx.Output_Sum else Get_PWM (Heater)),
               Last_Temperature            => (if Ctx.Kind = PID_Kind then Ctx.Last_Temperature else -1_000_000.0));
      end case;

      Global_Update_Blocker := False;
   end Setup;

   procedure Set_Setpoint (Heater : Heater_Name; Setpoint : Celcius) is
   begin
      pragma Assert (not Global_Update_Blocker);
      --  We do not need to hold the update blocker here as the Setpoint component is atomic, however the setup
      --  procedures must not be called at the same time, hence neither is allowed to be called from an ISR.

      Contexts (Heater).Setpoint := Setpoint;
   end Set_Setpoint;

   procedure Update (Heater : Heater_Name; Current_Temperature : Celcius) is
      Ctx : Context renames Contexts (Heater);
   begin
      if Global_Update_Blocker then
         return;
         --  This means that we skip updates when any heater parameters are being updated. This should not be
         --  problematic under normal circumstances.
      end if;

      case Ctx.Kind is
         when Disabled_Kind =>
            Set_PWM (Heater, 0.0);
         when Bang_Bang_Kind =>
            if Current_Temperature > Ctx.Setpoint + Ctx.Hysteresis then
               Set_PWM (Heater, 0.0);
            elsif Current_Temperature < Ctx.Setpoint + Ctx.Hysteresis then
               Set_PWM (Heater, 1.0);
            end if;
         when PID_Kind =>
            if Ctx.Last_Temperature = -1_000_000.0 then
               --  Heater has only just been switched to PID, or something is broken.
               Ctx.Last_Temperature := Current_Temperature;
            end if;
            declare
               Error   : constant Celcius := Ctx.Setpoint - Current_Temperature;
               Delta_T : constant Celcius := Current_Temperature - Ctx.Last_Temperature;
               Output  : Float;
            begin
               Ctx.Output_Sum := @ + (Ctx.Integral_Scale * Error);

               if Ctx.Proportional_On_Measurement then
                  Ctx.Output_Sum := @ - (Ctx.Proportional_Scale * Error);
                  Output         := 0.0;
               else
                  Output := Ctx.Proportional_Scale * Error;
               end if;

               if Ctx.Output_Sum < 0.0 then
                  Ctx.Output_Sum := 0.0;
               elsif Ctx.Output_Sum > 1.0 then
                  Ctx.Output_Sum := 1.0;
               end if;

               Output := @ - Ctx.Derivative_Scale * Delta_T;

               if Output < 0.0 then
                  Output := 0.0;
               elsif Output > 1.0 then
                  Output := 1.0;
               end if;

               Set_PWM (Heater, Output);

               Ctx.Last_Temperature := Current_Temperature;
            end;
      end case;

      Updated_Heaters (Heater) := True;
      if Updated_Heaters = Updated_Heaters_Type'(others => True) then
         Updated_Heaters := (others => False);
         STM32.IWDG.Reset_Watchdog;
      end if;

      if Ctx.Kind = Disabled_Kind then
         return;
      end if;

      --  Algorithm from Klipper.
      if Current_Temperature >= Ctx.Setpoint - Ctx.Hysteresis or Ctx.Setpoint <= 0.0 then
         Ctx.Approaching_Setpoint := False;
         Ctx.Starting_Approach    := False;
         if Current_Temperature <= Ctx.Setpoint + Ctx.Hysteresis then
            Ctx.Cumulative_Error := 0.0;
         end if;
         Ctx.Last_Setpoint := Ctx.Setpoint;
      else
         Ctx.Cumulative_Error := @ + (Ctx.Setpoint - Ctx.Hysteresis) - Current_Temperature;
         if not Ctx.Approaching_Setpoint then
            if Ctx.Setpoint /= Ctx.Last_Setpoint then
               Ctx.Approaching_Setpoint := True;
               Ctx.Starting_Approach    := True;
               Ctx.Check_Goal_Temp      := Current_Temperature + Ctx.Check_Minimum_Gain;
               Ctx.Check_Goal_Time      := Clock + Ctx.Check_Gain_Time;
            elsif Ctx.Cumulative_Error > Ctx.Max_Cumulative_Error then
               Make_Safe;
               Server_Communication.Transmit_String_Line ("Heater " & Heater'Image & " could not maintain setpoint.");
               Server_Communication.Transmit_Fatal_Exception_Mark;
            end if;
         elsif Current_Temperature >= Ctx.Check_Goal_Temp then
            Ctx.Starting_Approach := False;
            Ctx.Cumulative_Error  := 0.0;
            Ctx.Check_Goal_Temp   := Current_Temperature + Ctx.Check_Minimum_Gain;
            Ctx.Check_Goal_Time   := Clock + Ctx.Check_Gain_Time;
         elsif Clock >= Ctx.Check_Goal_Time then
            Ctx.Approaching_Setpoint := False;
         elsif Ctx.Starting_Approach then
            Ctx.Check_Goal_Temp := Celcius'Min (Ctx.Check_Goal_Temp, Current_Temperature + Ctx.Check_Minimum_Gain);
         end if;
         Ctx.Last_Setpoint := Ctx.Setpoint;
      end if;
   end Update;

   procedure Wait_Until_Stable (Heater : Heater_Name) is
      Ctx : Context renames Contexts (Heater);
   begin
      case Ctx.Kind is
         when Disabled_Kind =>
            null;
         when Bang_Bang_Kind =>
            loop
               exit when Ctx.Last_Temperature - Ctx.Setpoint < Ctx.Hysteresis;
            end loop;
         when PID_Kind =>
            declare
               Last_Bad_Time : Time := Clock;
            begin
               loop
                  if abs (Ctx.Last_Temperature - Ctx.Setpoint) > 5.0 then
                     Last_Bad_Time := Clock;
                  end if;
                  exit when Clock > Last_Bad_Time + Seconds (3);
               end loop;
            end;
      end case;
   end Wait_Until_Stable;

   procedure Set_PWM (Heater : Heater_Name; Scale : Float) is
   begin
      Set_Compare_Value (Heater_Timers (Heater).all, Heater_Timer_Channels (Heater), UInt16 (Scale * 50_001.0));
   end Set_PWM;

   function Get_PWM (Heater : Heater_Name) return Float is
   begin
      return
        Float (UInt16'(Current_Capture_Value (Heater_Timers (Heater).all, Heater_Timer_Channels (Heater)))) / 50_001.0;
   end Get_PWM;

   function Get_PWM (Heater : Heater_Name) return Fixed_Point_PWM_Scale is
   begin
      return Fixed_Point_PWM_Scale (Float'(Get_PWM (Heater)));
   end Get_PWM;

   procedure Start_Watchdog is
   begin
      STM32.IWDG.Initialize_Watchdog (STM32.IWDG.Divider_32, 4_000); --  Approximately 4 seconds.
      STM32.IWDG.Start_Watchdog;
   end Start_Watchdog;

end Heaters;
