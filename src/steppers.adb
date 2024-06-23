with STM32.USARTs;           use STM32.USARTs;
with HAL;                    use HAL;
with Ada.Real_Time;          use Ada.Real_Time;
with Hardware_Configuration; use Hardware_Configuration;
with STM32.GPIO;             use STM32.GPIO;
with STM32.Device;           use STM32.Device;
with Server_Communication;

package body Steppers is

   procedure Init is
   begin
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
      RX_Buffer : array (1 .. 12) of UInt8 := (others => 0);
      --  Extra bytes for transmitted bytes since we are using half-duplex mode.
      Fail_Time : Time;
   begin
      Receive_Failed := False;

      for S in USART_Status_Flag loop
         Clear_Status (TMC_UART, S);
      end loop;

      while Rx_Ready (TMC_UART) or TMC_UART_Internal.ISR.BUSY loop
         declare
            Junk : UInt9 := TMC_UART_Internal.RDR.RDR;
         begin
            Server_Communication.Transmit_String_Line
              ("Unexpected data on TMC UART before read (" & Junk'Image & ").");
         end;
      end loop;

      for S in USART_Status_Flag loop
         Clear_Status (TMC_UART, S);
      end loop;

      --  STM32G474 has a 8 byte FIFO (Table 345, RM0440 Rev 8), so no need for DMA here.
      TMC_UART_Internal.CR1.TE := False;
      for Byte of Input loop
         Transmit (TMC_UART, UInt9 (Byte));
      end loop;
      TMC_UART_Internal.CR1.TE := True;

      Fail_Time := Clock + Seconds (1);

      Outer :
      for I in RX_Buffer'Range loop
         loop
            exit when Rx_Ready (TMC_UART);
            if Clock > Fail_Time then
               Receive_Failed := True;
               exit Outer;
            end if;
         end loop;
         RX_Buffer (I) := UInt8 (Current_Input (TMC_UART));
      end loop Outer;

      if Status (TMC_UART, Overrun_Error_Indicated) then
         Receive_Failed := True;
         Server_Communication.Transmit_String_Line ("TMC UART overrun.");
      end if;

      --  TODO: DMA appears to not work when HDSEL is set, but this is entirely undocumented. As transmitted bytes go
      --  in to the 8-byte receiving FIFO, we can not rely on the FIFO to guarantee that all bytes will be received.

      for I in 1 .. 8 loop
         Output (I) := TMC2240_UART_Byte (RX_Buffer (I + 4));
      end loop;
   end UART_Read;

   procedure UART_Write (Input : TMC2240_UART_Data_Byte_Array) is
   begin
      for S in USART_Status_Flag loop
         Clear_Status (TMC_UART, S);
      end loop;

      while Rx_Ready (TMC_UART) or TMC_UART_Internal.ISR.BUSY loop
         declare
            Junk : UInt9 := TMC_UART_Internal.RDR.RDR;
         begin
            Server_Communication.Transmit_String_Line
              ("Unexpected data on TMC UART before write (" & Junk'Image & ").");
         end;
      end loop;

      for S in USART_Status_Flag loop
         Clear_Status (TMC_UART, S);
      end loop;

      --  STM32G474 has a 8 byte FIFO (Table 345, RM0440 Rev 8), so no need for DMA here.
      TMC_UART_Internal.CR1.TE := False;
      for Byte of Input loop
         Transmit (TMC_UART, UInt9 (Byte));
      end loop;
      TMC_UART_Internal.CR1.TE := True;
   end UART_Write;

end Steppers;
