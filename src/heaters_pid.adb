with Ada.Numerics;
with Server_Communication;
with Heaters;

package body Heaters_PID is

   procedure Autotune_Update
     (Setpoint : in out Temperature; Ctx : in out Context; Current_Temperature : Temperature; PWM : out PWM_Scale)
   is
      Start_Time : constant Ada.Real_Time.Time := Clock;
   begin
      --  Algorithm from Marlin.
      if (Current_Temperature > Setpoint + 30.0 * celcius) then
         Heaters.Make_Safe;
         Server_Communication.Transmit_String_Line ("Heater overshot by over 30 C during PID autotune.");
         Server_Communication.Transmit_Fatal_Exception_Mark;
      elsif Start_Time - Ctx.Autotune.T1 > Minutes (20) and Start_Time - Ctx.Autotune.T2 > Minutes (20) then
         Heaters.Make_Safe;
         Server_Communication.Transmit_String_Line ("Heater has taken over 20 minutes to cycle during PID autotune.");
         Server_Communication.Transmit_Fatal_Exception_Mark;
      end if;

      Ctx.Autotune.Max_T := Temperature'Max (@, Current_Temperature);
      Ctx.Autotune.Min_T := Temperature'Min (@, Current_Temperature);

      if Ctx.Autotune.Heating and Current_Temperature > Setpoint and Start_Time > Ctx.Autotune.T2 + Seconds (5) then
         Ctx.Autotune.Heating := False;
         PWM                  := (Ctx.Autotune.Bias - Ctx.Autotune.D) / 2.0;
         Ctx.Autotune.T1      := Start_Time;
         Ctx.Autotune.T_High  := Ctx.Autotune.T1 - Ctx.Autotune.T2;
         Ctx.Autotune.Max_T   := Setpoint;
      elsif (not Ctx.Autotune.Heating) and Current_Temperature < Setpoint and
        Start_Time > Ctx.Autotune.T1 + Seconds (5)
      then
         Ctx.Autotune.Heating := True;
         Ctx.Autotune.T2      := Start_Time;
         Ctx.Autotune.T_Low   := Ctx.Autotune.T2 - Ctx.Autotune.T1;
         if Ctx.Autotune.Cycles > 0 then
            Ctx.Autotune.Bias :=
              @ +
              (Ctx.Autotune.D * Physical_Types.Time (To_Duration (Ctx.Autotune.T_High - Ctx.Autotune.T_Low)) /
               Physical_Types.Time (To_Duration (Ctx.Autotune.T_High + Ctx.Autotune.T_Low)));
            Ctx.Autotune.Bias := PWM_Scale'Min (0.92, PWM_Scale'Max (0.08, @));
            if Ctx.Autotune.Bias > 0.5 then
               Ctx.Autotune.D := 0.995 - Ctx.Autotune.Bias;
            else
               Ctx.Autotune.D := Ctx.Autotune.Bias;
            end if;

            Server_Communication.Transmit_String_Line
              ("Bias=" & Ctx.Autotune.Bias'Image & " D=" & Ctx.Autotune.D'Image & " Max_T=" &
               Ctx.Autotune.Max_T'Image & " Min_T=" & Ctx.Autotune.Min_T'Image);

            if Ctx.Autotune.Cycles > 2 then
               declare
                  Ku : constant Inverse_Temperature :=
                    8.0 * Ctx.Autotune.D / (Ada.Numerics.Pi * (Ctx.Autotune.Max_T - Ctx.Autotune.Min_T));
                  Tu : constant Physical_Types.Time :=
                    Dimensionless (To_Duration (Ctx.Autotune.T_Low + Ctx.Autotune.T_High)) * s;
               begin
                  Ctx.Autotune.Proportional_Scale := Ku * Ctx.Autotune.Pf;
                  Ctx.Autotune.Integral_Scale     := Ctx.Autotune.Proportional_Scale * (2.0 * s / Tu);
                  Ctx.Autotune.Derivative_Scale   := Ctx.Autotune.Proportional_Scale * Tu * Ctx.Autotune.Df;

                  Server_Communication.Transmit_String_Line ("Ku=" & Ku'Image & " Tu=" & Tu'Image);
                  Server_Communication.Transmit_String_Line
                    ("P=" & Ctx.Autotune.Proportional_Scale'Image & " I=" & Ctx.Autotune.Integral_Scale'Image & " D=" &
                     Ctx.Autotune.Derivative_Scale'Image);
               end;
            end if;
         end if;

         PWM := (Ctx.Autotune.Bias + Ctx.Autotune.D) / 2.0;

         Server_Communication.Transmit_String_Line (Ctx.Autotune.Cycles'Image & "/" & Ctx.Autotune.Max_Cycles'Image);

         Ctx.Autotune.Cycles := @ + 1;
         Ctx.Autotune.Min_T  := Setpoint;
      end if;

      if Ctx.Autotune.Cycles > Natural'Max (2, Ctx.Autotune.Max_Cycles) then
         PWM                  := 0.0;
         Setpoint             := 0.0 * celcius;
         Ctx.In_Autotune_Mode := False;
         Server_Communication.Transmit_String_Line ("PID autotune done.");
      end if;
   end Autotune_Update;

   procedure Update
     (Setpoint : in out Temperature; Ctx : in out Context; Current_Temperature : Temperature; PWM : out PWM_Scale)
   is
   begin
      if Ctx.In_Autotune_Mode then
         Autotune_Update (Setpoint, Ctx, Current_Temperature, PWM);
      else
         if Ctx.Last_Temperature = -1_000_000.0 then
            --  Heater has only just been switched to PID, or something is broken.
            Ctx.Last_Temperature := Current_Temperature;
         end if;
         declare
            Error   : constant Temperature := Setpoint - Current_Temperature;
            Delta_T : constant Temperature := Current_Temperature - Ctx.Last_Temperature;
            Output  : Dimensionless        := Ctx.Proportional_Scale * Error;
         begin
            Ctx.Output_Sum := @ + (Ctx.Integral_Scale * Error);

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

            PWM := Output;

            Ctx.Last_Temperature := Current_Temperature;
         end;
      end if;
   end Update;

   procedure Start_Autotune (Setpoint : in out Temperature; Ctx : in out Context; PWM : out PWM_Scale) is
      Start_Time : constant Ada.Real_Time.Time := Clock;
   begin
      if Ctx.In_Autotune_Mode then
         raise Constraint_Error with "PID auotune already running.";
      end if;

      Ctx.Autotune :=
        (Bias               => 0.5, --  Half of max power (always 1.0 for now).
         D                  => 0.5, --  Half of max power (always 1.0 for now).
         T1                 => Start_Time,
         T2                 => Start_Time,
         T_High             => Seconds (0),
         T_Low              => Seconds (0),
         Cycles             => 0,
         Heating            => True,
         Max_T              => 0.0 * celcius,
         Min_T              => 10_000.0 * celcius,
         Max_Cycles         => 5,
         Pf                 => 0.6, --  TODO: 0.2 for bed or chamber.
         Df                 => 0.125 * hertz, --  TODO: 1/3 for bed or chamber.
         Proportional_Scale => 0.0 / celcius,
         Integral_Scale     => 0.0 / celcius,
         Derivative_Scale   => 0.0 / celcius);

      PWM := Ctx.Autotune.Bias;

      Ctx.In_Autotune_Mode := True; --  Must be set last as we do not hold a lock here.
   end Start_Autotune;

end Heaters_PID;
