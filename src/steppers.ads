with Messages; use Messages;

package Steppers is

   procedure Init;
   procedure Enable (Stepper : Stepper_Name);
   procedure Disable (Stepper : Stepper_Name);
   procedure UART_Read
     (Input          :     TMC2240_UART_Query_Byte_Array;
      Receive_Failed : out Byte_Boolean;
      Output         : out TMC2240_UART_Data_Byte_Array);
   procedure UART_Write (Input : TMC2240_UART_Data_Byte_Array);

end Steppers;
