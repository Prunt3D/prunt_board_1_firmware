with Ada.Real_Time;  use Ada.Real_Time;
with Physical_Types; use Physical_Types;
limited with Heaters;

package Heaters_PID is

   subtype PID_Scale is Inverse_Temperature range 0.0 .. Dimensionless'Last;

   type Autotune_Context is record
      Setpoint           : Temperature;
      Bias               : PWM_Scale;
      D                  : PWM_Scale;
      T1                 : Ada.Real_Time.Time;
      T2                 : Ada.Real_Time.Time;
      T_High             : Time_Span;
      T_Low              : Time_Span;
      Cycles             : Natural;
      Heating            : Boolean;
      Max_T              : Temperature;
      Min_T              : Temperature;
      Max_Cycles         : Natural;
      Pf                 : Dimensionless;
      Df                 : Frequency;
      Proportional_Scale : PID_Scale;
      Integral_Scale     : PID_Scale;
      Derivative_Scale   : PID_Scale;
   end record with
     Volatile;

   type Context is record
      Proportional_Scale : PID_Scale;
      Integral_Scale     : PID_Scale;
      Derivative_Scale   : PID_Scale;
      Output_Sum         : Dimensionless;
      Last_Temperature   : Temperature;
      In_Autotune_Mode   : Boolean with
        Atomic;
      Autotune           : Autotune_Context;
   end record with
     Volatile;

   procedure Update
     (Setpoint : Temperature; Ctx : in out Context; Current_Temperature : Temperature; PWM : out PWM_Scale);
   procedure Start_Autotune (Setpoint : Temperature; Ctx : in out Context; PWM : out PWM_Scale);

private

   procedure Autotune_Update (Ctx : in out Context; Current_Temperature : Temperature; PWM : out PWM_Scale);

end Heaters_PID;
