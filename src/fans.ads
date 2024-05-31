with Messages; use Messages;

package Fans is

   procedure Init;
   procedure Set_PWM (Fan : Fan_Name; Scale : Fixed_Point_PWM_Scale);

end Fans;
