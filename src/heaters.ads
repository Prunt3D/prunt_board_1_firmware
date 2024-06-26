--  PID algorithm from https://github.com/br3ttb/Arduino-PID-Library
--  Copyright Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com

with Messages;               use Messages;
with Hardware_Configuration; use Hardware_Configuration;
with Ada.Real_Time;          use Ada.Real_Time;
with Physical_Types;         use Physical_Types;

package Heaters is

   procedure Init;

   procedure Make_Safe;
   --  May be called before Init.

   procedure Setup (Heater : Heater_Name; Parameters : Heater_Parameters);
   --  Must not be called from ISRs.

   procedure Set_Setpoint (Heater : Heater_Name; Setpoint : Temperature);
   --  Must not be called from ISRs.

   procedure Update (Heater : Heater_Name; Current_Temperature : Temperature);

   function Get_PWM (Heater : Heater_Name) return Fixed_Point_PWM_Scale;

   procedure Wait_Until_Stable (Heater : Heater_Name);

   procedure Start_Watchdog;
   --  Should only be called once. Calling again will reset the timer.

private

   subtype PID_Scale is Inverse_Temperature range 0.0 .. Dimensionless'Last;

   procedure Set_PWM (Heater : Heater_Name; Scale : PWM_Scale) with
     Pre => Scale >= 0.0 and Scale <= 1.0;
   function Get_PWM (Heater : Heater_Name) return Dimensionless with
     Post => Get_PWM'Result >= 0.0 and Get_PWM'Result <= 1.0;

   type Context (Kind : Heater_Kind := Disabled_Kind) is record
      Setpoint                   : Temperature with
        Atomic;
      Check_Max_Cumulative_Error : Temperature;
      Check_Gain_Time            : Time_Span;
      Check_Minimum_Gain         : Temperature;
      Check_Hysteresis           : Temperature;
      Check_Approaching_Setpoint : Boolean;
      Check_Starting_Approach    : Boolean;
      Check_Cumulative_Error     : Temperature;
      Check_Last_Setpoint        : Temperature;
      Check_Goal_Temp            : Temperature;
      Check_Goal_Time            : Ada.Real_Time.Time;
      case Kind is
         when Disabled_Kind =>
            null;
         when Bang_Bang_Kind =>
            null;
         when PID_Kind =>
            Proportional_Scale : PID_Scale;
            Integral_Scale     : PID_Scale;
            Derivative_Scale   : PID_Scale;
            Output_Sum         : Dimensionless;
            Last_Temperature   : Temperature;
      end case;
   end record with
     Volatile;

   Contexts : array (Heater_Name) of Context := (others => (Kind => Disabled_Kind, others => <>)) with
     Volatile_Components;

   Global_Update_Blocker : Boolean := False with
     Atomic, Volatile;

   type Updated_Heaters_Type is array (Heater_Name) of Boolean with
     Volatile_Components, Atomic_Components;
   Updated_Heaters : Updated_Heaters_Type with
     Volatile, Atomic;
   --  It should be safe to remove this Atomic on boards with too many heaters for it to be an option.

end Heaters;
