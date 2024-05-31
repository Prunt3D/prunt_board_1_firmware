--  PID algorithm from https://github.com/br3ttb/Arduino-PID-Library
--  Copyright Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com

with Messages;               use Messages;
with Hardware_Configuration; use Hardware_Configuration;

package Heaters is

   subtype Celcius is Float;

   procedure Init;

   procedure Make_Safe;
   --  May be called before Init.

   procedure Setup (Heater : Heater_Name; Parameters : Heater_Parameters);
   --  Must not be called from ISRs.

   procedure Set_Setpoint (Heater : Heater_Name; Setpoint : Celcius);
   --  Must not be called from ISRs.

   procedure Update (Heater : Heater_Name; Current_Temperature : Celcius);

   function Get_PWM (Heater : Heater_Name) return Fixed_Point_PWM_Scale;

private

   subtype PID_Scale is Float range 0.0 .. Float'Last;

   procedure Set_PWM (Heater : Heater_Name; Scale : Float) with
     Pre => Scale >= 0.0 and Scale <= 1.0;
   function Get_PWM (Heater : Heater_Name) return Float with
     Post => Get_PWM'Result >= 0.0 and Get_PWM'Result <= 1.0;

   type Context (Kind : Heater_Kind := Disabled_Kind) is record
      Setpoint : Celcius with
        Atomic;
      case Kind is
         when Disabled_Kind =>
            null;
         when Bang_Bang_Kind =>
            Max_Delta : Celcius;
         when PID_Kind =>
            Proportional_Scale          : PID_Scale;
            Integral_Scale              : PID_Scale;
            Derivative_Scale            : PID_Scale;
            Proportional_On_Measurement : Boolean;
            Output_Sum                  : Float;
            Last_Temperature            : Celcius;
      end case;
   end record with
     Volatile;

   Contexts : array (Heater_Name) of Context := (others => (Kind => Disabled_Kind, Setpoint => 0.0)) with
     Volatile_Components;

   Global_Update_Blocker : Boolean := False with
     Atomic, Volatile;

end Heaters;
