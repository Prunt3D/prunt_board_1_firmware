with Physical_Types; use Physical_Types;
with Messages;       use Messages;
limited with Heaters;
with Step_Generator;
with Ada.Real_Time;

package Heaters_MPC is

   type Autotune_Stage is (Measure_Ambient, Heatup_Test, First_Pass, Transfer_Test, Second_Pass);

   type Heatup_Samples_Array is array (1 .. 3) of Temperature;

   type Autotune_Context is record
      State                    : Autotune_Stage;
      Setpoint                 : Temperature;
      Use_Analytic             : Boolean;
      Ambient_Max_Measure_Time : Physical_Types.Time;
      Ambient_Max_Sample_Time  : Physical_Types.Time;
      Threshold_Temp           : Temperature;
      Ambient_Temp             : Temperature;
      Heatup_Samples           : Heatup_Samples_Array;
      Heatup_Sample_Gap_Time   : Ada.Real_Time.Time_Span;
      Block_Responsiveness     : Frequency;
      Ambient_Transfer         : Specific_Heat_Transfer_Coefficient;
      Block_Heat_Capacity      : Heat_Capacity;
      Sensor_Responsiveness    : Frequency;
      --  Transfer test next.
   end record;

   type Context is record
      Block_Heat_Capacity        : Heat_Capacity;
      Ambient_Transfer           : Specific_Heat_Transfer_Coefficient;
      Full_Fan_Ambient_Transfer  : Specific_Heat_Transfer_Coefficient;
      --  TODO: Support an array of fan transfer coefficients.
      Target_Reach_Time          : Physical_Types.Time;
      Heater_Power               : Power;
      --  TODO: Heater power as a function of temperature.
      Smoothing                  : Smoothing_Factor;
      Sensor_Responsiveness      : Frequency;
      Min_Ambient_Change         : Dimensionless;
      Steady_State_Rate          : Temperature_Over_Time;
      Filament_Heat_Capacity     : Lengthwise_Heat_Capacity;
      Extruder_Distance_Per_Step : Length;
      Last_Extruder_Vel          : Velocity;
      Last_Extruder_Steps        : Step_Generator.Big_Step_Count;
      Cooling_Fan                : Fan_Name;
      Last_Power                 : Power;
      State_Block_Temp           : Temperature;
      State_Ambient_Temp         : Temperature;
      State_Sensor_Temp          : Temperature;
      In_Autotune_Mode           : Boolean;
   end record with
     Volatile;

   procedure Update
     (Setpoint : Temperature; Ctx : in out Context; Current_Temperature : Temperature; PWM : out PWM_Scale);
   procedure Start_Autotune (Setpoint : Temperature; Ctx : in out Context; PWM : out PWM_Scale);

end Heaters_MPC;
