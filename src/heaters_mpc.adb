with Fans;
with Thermistors;

--  Based on Danger Klipper implementation:
--  https://github.com/DangerKlippers/danger-klipper/blob/10002c97f87ed19dc80d05dd6dfd917c88ff535f/klippy/extras/control_mpc.py
package body Heaters_MPC is

   procedure Update
     (Setpoint : Temperature; Ctx : in out Context; Current_Temperature : Temperature; PWM : out PWM_Scale)
   is
      use type Step_Generator.Big_Step_Count;

      Extruder_Steps    : constant Step_Generator.Big_Step_Count := Step_Generator.Get_Extruder_Total_Steps;
      Extruder_Velocity : constant Velocity                      :=
        Dimensionless (Ctx.Last_Extruder_Steps - Extruder_Steps) * Ctx.Extruder_Distance_Per_Step *
        Thermistors.Loop_Frequency;

      Total_Ambient_Transfer_Coef : constant Specific_Heat_Transfer_Coefficient :=
        Ctx.Ambient_Transfer + Ctx.Full_Fan_Ambient_Transfer * Dimensionless (Fans.Get_PWM (Ctx.Cooling_Fan));

      Block_Ambient_Delta : constant Temperature := Ctx.State_Block_Temp - Ctx.State_Ambient_Temp;

      Expected_Ambient_Transfer  : constant Power := Block_Ambient_Delta * Total_Ambient_Transfer_Coef;
      Expected_Filament_Transfer : constant Power :=
        Block_Ambient_Delta * Extruder_Velocity * Ctx.Filament_Heat_Capacity;

      Expected_Block_Delta_Temp : constant Temperature :=
        (Ctx.Last_Power - Expected_Ambient_Transfer - Expected_Filament_Transfer) /
        (Thermistors.Loop_Frequency * Ctx.Block_Heat_Capacity);

      Expected_Sensor_Delta_Temp : Temperature;
      Adjustment_Delta_Temp      : Temperature;
   begin
      Ctx.State_Block_Temp := Ctx.State_Block_Temp + Expected_Block_Delta_Temp;

      Expected_Sensor_Delta_Temp :=
        (Ctx.State_Block_Temp - Ctx.State_Sensor_Temp) * Ctx.Sensor_Responsiveness / Thermistors.Loop_Frequency;
      Ctx.State_Sensor_Temp      := Ctx.State_Sensor_Temp + Expected_Sensor_Delta_Temp;

      --  Difference from Danger Klipper implementation: We do not scale the smoothing factor here since we assume that
      --  the update period is constant.
      Adjustment_Delta_Temp := (Current_Temperature - Ctx.State_Sensor_Temp) * Ctx.Smoothing;
      Ctx.State_Block_Temp  := Ctx.State_Block_Temp + Adjustment_Delta_Temp;
      Ctx.State_Sensor_Temp := Ctx.State_Sensor_Temp + Adjustment_Delta_Temp;

      if (Ctx.Last_Power > 0.0 * watt and Ctx.Last_Power < Ctx.Heater_Power) or
        abs (Expected_Block_Delta_Temp + Adjustment_Delta_Temp) < Ctx.Steady_State_Rate / Thermistors.Loop_Frequency
      then
         if Adjustment_Delta_Temp > 0.0 then
            Ctx.State_Ambient_Temp :=
              Ctx.State_Ambient_Temp +
              Temperature'Max (Adjustment_Delta_Temp, Ctx.Min_Ambient_Change / Thermistors.Loop_Frequency);
         else
            Ctx.State_Ambient_Temp :=
              Ctx.State_Ambient_Temp +
              Temperature'Min (Adjustment_Delta_Temp, -Ctx.Min_Ambient_Change / Thermistors.Loop_Frequency);
         end if;
      end if;

      declare
         Heating_Power       : constant Power       :=
           (Setpoint - Ctx.State_Block_Temp) * Ctx.Block_Heat_Capacity / Ctx.Target_Reach_Time;
         Block_Ambient_Delta : constant Temperature := Ctx.State_Block_Temp - Ctx.State_Ambient_Temp;
         Loss_Ambient        : constant Power       := Block_Ambient_Delta * Ctx.Ambient_Transfer;
         Loss_Filament       : constant Power := Block_Ambient_Delta * Extruder_Velocity * Ctx.Filament_Heat_Capacity;
         New_Power           : constant Power       :=
           Power'Max (0.0, Power'Min (Ctx.Heater_Power, Heating_Power + Loss_Ambient + Loss_Filament));
      begin
         Ctx.Last_Power          := New_Power;
         Ctx.Last_Extruder_Steps := Extruder_Steps;
         Ctx.Last_Extruder_Vel   := Extruder_Velocity;

         PWM := New_Power / Ctx.Heater_Power;
      end;
   end Update;

   procedure Start_Autotune (Setpoint : Temperature; Ctx : in out Context; PWM : out PWM_Scale) is null;

end Heaters_MPC;
