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
         Setup
           (Heater,
            (Kind                 => Disabled_Kind,
             Max_Cumulative_Error => 0.0,
             Check_Gain_Time      => 0.0,
             Check_Minimum_Gain   => 0.0,
             Hysteresis           => 0.0));

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
              (Kind                       => Disabled_Kind,
               Setpoint                   => 0.0 * celcius,
               Check_Max_Cumulative_Error => Dimensionless (Parameters.Max_Cumulative_Error) * celcius,
               Check_Gain_Time            => To_Time_Span (Duration (Parameters.Check_Gain_Time)),
               Check_Minimum_Gain         => Dimensionless (Parameters.Check_Minimum_Gain) * celcius,
               Check_Hysteresis           => Dimensionless (Parameters.Hysteresis) * celcius,
               Check_Approaching_Setpoint => False,
               Check_Starting_Approach    => False,
               Check_Cumulative_Error     => 0.0 * celcius,
               Check_Last_Setpoint        => 0.0 * celcius,
               Check_Goal_Temp            => 0.0 * celcius,
               Check_Goal_Time            => <>);
         when Bang_Bang_Kind =>
            Ctx :=
              (Kind                       => Bang_Bang_Kind,
               Setpoint                   => Ctx.Setpoint,
               Check_Max_Cumulative_Error => Dimensionless (Parameters.Max_Cumulative_Error) * celcius,
               Check_Gain_Time            => To_Time_Span (Duration (Parameters.Check_Gain_Time)),
               Check_Minimum_Gain         => Dimensionless (Parameters.Check_Minimum_Gain) * celcius,
               Check_Hysteresis           => Dimensionless (Parameters.Hysteresis) * celcius,
               Check_Approaching_Setpoint => False,
               Check_Starting_Approach    => False,
               Check_Cumulative_Error     => 0.0 * celcius,
               Check_Last_Setpoint        => 0.0 * celcius,
               Check_Goal_Temp            => 0.0 * celcius,
               Check_Goal_Time            => <>);
         when PID_Kind =>
            Ctx :=
              (Kind                       => PID_Kind,
               Setpoint                   => Ctx.Setpoint,
               Check_Max_Cumulative_Error => Dimensionless (Parameters.Max_Cumulative_Error) * celcius,
               Check_Gain_Time            => To_Time_Span (Duration (Parameters.Check_Gain_Time)),
               Check_Minimum_Gain         => Dimensionless (Parameters.Check_Minimum_Gain) * celcius,
               Check_Hysteresis           => Dimensionless (Parameters.Hysteresis) * celcius,
               Check_Approaching_Setpoint => False,
               Check_Starting_Approach    => False,
               Check_Cumulative_Error     => 0.0 * celcius,
               Check_Last_Setpoint        => 0.0 * celcius,
               Check_Goal_Temp            => 0.0 * celcius,
               Check_Goal_Time            => <>,
               Proportional_Scale         => PID_Scale (Parameters.Proportional_Scale),
               Integral_Scale             => PID_Scale (Parameters.Integral_Scale),
               Derivative_Scale           => PID_Scale (Parameters.Derivative_Scale),
               Output_Sum                 => (if Ctx.Kind = PID_Kind then Ctx.Output_Sum else Get_PWM (Heater)),
               Last_Temperature => (if Ctx.Kind = PID_Kind then Ctx.Last_Temperature else -1_000_000.0 * celcius));
      end case;

      Global_Update_Blocker := False;
   end Setup;

   procedure Set_Setpoint (Heater : Heater_Name; Setpoint : Temperature) is
   begin
      pragma Assert (not Global_Update_Blocker);
      --  We do not need to hold the update blocker here as the Setpoint component is atomic, however the setup
      --  procedures must not be called at the same time, hence neither is allowed to be called from an ISR.

      Contexts (Heater).Setpoint := Setpoint;
   end Set_Setpoint;

   procedure Update (Heater : Heater_Name; Current_Temperature : Temperature) is
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
            if Current_Temperature > Ctx.Setpoint + Ctx.Check_Hysteresis then
               Set_PWM (Heater, 0.0);
            elsif Current_Temperature < Ctx.Setpoint + Ctx.Check_Hysteresis then
               Set_PWM (Heater, 1.0);
            end if;
         when PID_Kind =>
            if Ctx.Last_Temperature = -1_000_000.0 then
               --  Heater has only just been switched to PID, or something is broken.
               Ctx.Last_Temperature := Current_Temperature;
            end if;
            declare
               Error   : constant Temperature := Ctx.Setpoint - Current_Temperature;
               Delta_T : constant Temperature := Current_Temperature - Ctx.Last_Temperature;
               Output  : Dimensionless;
            begin
               Ctx.Output_Sum := @ + (Ctx.Integral_Scale * Error);

               Output := Ctx.Proportional_Scale * Error;

               if Ctx.Output_Sum < 0.0 then
                  Ctx.Output_Sum := 0.0;
               elsif Ctx.Output_Sum > 1.0 then
                  Ctx.Output_Sum := 1.0;
               end if;

               Output := @ + Ctx.Output_Sum - Ctx.Derivative_Scale * Delta_T;

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
      if Current_Temperature >= Ctx.Setpoint - Ctx.Check_Hysteresis or Ctx.Setpoint <= 0.0 then
         Ctx.Check_Approaching_Setpoint := False;
         Ctx.Check_Starting_Approach    := False;
         if Current_Temperature <= Ctx.Setpoint + Ctx.Check_Hysteresis then
            Ctx.Check_Cumulative_Error := 0.0 * celcius;
         end if;
         Ctx.Check_Last_Setpoint := Ctx.Setpoint;
      else
         Ctx.Check_Cumulative_Error :=
           Ctx.Check_Cumulative_Error + (Ctx.Setpoint - Ctx.Check_Hysteresis) - Current_Temperature;
         if not Ctx.Check_Approaching_Setpoint then
            if Ctx.Setpoint /= Ctx.Check_Last_Setpoint then
               Ctx.Check_Approaching_Setpoint := True;
               Ctx.Check_Starting_Approach    := True;
               Ctx.Check_Goal_Temp            := Current_Temperature + Ctx.Check_Minimum_Gain;
               Ctx.Check_Goal_Time            := Clock + Ctx.Check_Gain_Time;
            elsif Ctx.Check_Cumulative_Error > Ctx.Check_Max_Cumulative_Error then
               Make_Safe;
               Server_Communication.Transmit_String_Line ("Heater " & Heater'Image & " could not maintain setpoint.");
               Server_Communication.Transmit_Fatal_Exception_Mark;
            end if;
         elsif Current_Temperature >= Ctx.Check_Goal_Temp then
            Ctx.Check_Starting_Approach := False;
            Ctx.Check_Cumulative_Error  := 0.0 * celcius;
            Ctx.Check_Goal_Temp         := Current_Temperature + Ctx.Check_Minimum_Gain;
            Ctx.Check_Goal_Time         := Clock + Ctx.Check_Gain_Time;
         elsif Clock >= Ctx.Check_Goal_Time then
            Ctx.Check_Approaching_Setpoint := False;
         elsif Ctx.Check_Starting_Approach then
            Ctx.Check_Goal_Temp := Temperature'Min (Ctx.Check_Goal_Temp, Current_Temperature + Ctx.Check_Minimum_Gain);
         end if;
         Ctx.Check_Last_Setpoint := Ctx.Setpoint;
      end if;
   end Update;

   function Check_If_Stable (Heater : Heater_Name) return Boolean is
      Ctx : Context renames Contexts (Heater);
   begin
      case Ctx.Kind is
         when Disabled_Kind =>
            return True;
         when Bang_Bang_Kind =>
            return abs (Ctx.Last_Temperature - Ctx.Setpoint) < Ctx.Check_Hysteresis;
         when PID_Kind =>
            return abs (Ctx.Last_Temperature - Ctx.Setpoint) < Ctx.Check_Hysteresis;
            --  TODO: Check if this is stable rather than just in-range.
      end case;
   end Check_If_Stable;

   procedure Set_PWM (Heater : Heater_Name; Scale : PWM_Scale) is
   begin
      Set_Compare_Value (Heater_Timers (Heater).all, Heater_Timer_Channels (Heater), UInt16 (Scale * 50_001.0));
   end Set_PWM;

   function Get_PWM (Heater : Heater_Name) return Dimensionless is
   begin
      return
        Dimensionless (UInt16'(Current_Capture_Value (Heater_Timers (Heater).all, Heater_Timer_Channels (Heater)))) /
        50_001.0;
   end Get_PWM;

   function Get_PWM (Heater : Heater_Name) return Fixed_Point_PWM_Scale is
   begin
      return Fixed_Point_PWM_Scale (Dimensionless'(Get_PWM (Heater)));
   end Get_PWM;

   procedure Start_Watchdog is
   begin
      STM32.IWDG.Initialize_Watchdog (STM32.IWDG.Divider_32, 4_000); --  Approximately 4 seconds.
      STM32.IWDG.Start_Watchdog;
   end Start_Watchdog;

end Heaters;
