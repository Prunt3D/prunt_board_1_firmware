with STM32.USARTs;           use STM32.USARTs;
with STM32.DMA;              use STM32.DMA;
with HAL;                    use HAL;
with Ada.Real_Time;          use Ada.Real_Time;
with Hardware_Configuration; use Hardware_Configuration;
with STM32.GPIO;             use STM32.GPIO;
with STM32.Device;           use STM32.Device;

package body Steppers is

   procedure Init is
   begin
      Enable_Clock (TMC_UART_DMA_RX_Controller);
      Enable_Clock (TMC_UART);

      Set_Word_Length (TMC_UART, Word_Length_8);
      Set_Parity (TMC_UART, No_Parity);
      Set_Mode (TMC_UART, Tx_Rx_Mode);
      Set_Oversampling_Mode (TMC_UART, Oversampling_By_16);
      Set_Stop_Bits (TMC_UART, Stopbits_1);
      Set_Flow_Control (TMC_UART, No_Flow_Control);
      Set_Baud_Rate (TMC_UART, 19_200);
      TMC_UART_Internal.CR1.FIFOEN := True;
      TMC_UART_Internal.CR3.HDSEL  := True;

      Enable (TMC_UART);

      delay until Clock + Milliseconds (10);

      Configure_IO
        (TMC_UART_Pin,
         (Mode           => Mode_AF,
          Resistors      => Floating,
          AF_Output_Type => Push_Pull,
          AF_Speed       => Speed_25MHz,
          AF             => TMC_UART_Pin_AF));

      delay until Clock + Milliseconds (10);

      Configure
        (TMC_UART_DMA_RX_Controller,
         TMC_UART_DMA_RX_Stream,
        (Channel                       => USART2_RX,
          Direction                    => Peripheral_To_Memory,
          Increment_Peripheral_Address => False,
          Increment_Memory_Address     => True,
          Peripheral_Data_Format       => Bytes,
          Memory_Data_Format           => Bytes,
          Operation_Mode               => Normal_Mode,
          Priority                     => TMC_UART_DMA_RX_Priority,
          Memory_Burst_Size            => Memory_Burst_Single,
          Peripheral_Burst_Size        => Peripheral_Burst_Single));

      Enable_DMA_Receive_Requests (TMC_UART);

      for S in Stepper_Name loop
         Disable (S);
         Configure_IO
           (Stepper_Enable_Points (S),
            (Mode => Mode_Out, Resistors => Floating, Output_Type => Push_Pull, Speed => Speed_100MHz));
      end loop;

      for S in Stepper_Name loop
         Configure_IO (Stepper_DIAG0_Points (S), (Mode => Mode_In, Resistors => Pull_Up));
      end loop;
   end Init;

   procedure Enable (Stepper : Stepper_Name) is
   begin
      Clear (Stepper_Enable_Points (Stepper));
   end Enable;

   procedure Disable (Stepper : Stepper_Name) is
   begin
      Set (Stepper_Enable_Points (Stepper));
   end Disable;

   procedure UART_Read
     (Input          :     TMC2240_UART_Query_Byte_Array;
      Receive_Failed : out Byte_Boolean;
      Output         : out TMC2240_UART_Data_Byte_Array)
   is
      type RX_Buffer_Type is array (1 .. 16) of UInt8 with
        Pack => True;
      RX_Buffer  : RX_Buffer_Type;
      DMA_Result : DMA_Error_Code;
   begin
      Start_Transfer
        (TMC_UART_DMA_RX_Controller,
         Stream      => TMC_UART_DMA_RX_Stream,
         Source      => Read_Data_Register_Address (TMC_UART),
         Destination => RX_Buffer'Address,
         Data_Count  => RX_Buffer'Length);

      --  STM32G474 has a 8 byte FIFO (Table 345, RM0440 Rev 8), so no need for DMA here.
      TMC_UART_Internal.CR1.TE := False;
      for Byte of Input loop
         Transmit (TMC_UART, UInt9 (Byte));
      end loop;
      TMC_UART_Internal.CR1.TE := True;

      Poll_For_Completion (TMC_UART_DMA_RX_Controller, TMC_UART_DMA_RX_Stream, Full_Transfer, Seconds (1), DMA_Result);

      if DMA_Result /= DMA_No_Error then
         Receive_Failed := True;
      else
         Receive_Failed := False;
         for I in 1 .. 8 loop
            Output (I) := TMC2240_UART_Byte (RX_Buffer (I + 8));
         end loop;
      end if;
   end UART_Read;

   procedure UART_Write (Input : TMC2240_UART_Data_Byte_Array) is
   begin
      --  STM32G474 has a 8 byte FIFO (Table 345, RM0440 Rev 8), so no need for DMA here.
      TMC_UART_Internal.CR1.TE := False;
      for Byte of Input loop
         Transmit (TMC_UART, UInt9 (Byte));
      end loop;
      TMC_UART_Internal.CR1.TE := True;
   end UART_Write;

end Steppers;
