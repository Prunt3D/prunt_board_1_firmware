with Messages; use Messages;

package Server_Communication is

   procedure Init;
   procedure Run;
   procedure Transmit_String (S : String);
   procedure Transmit_String_Line (S : String);
   procedure Transmit_Fatal_Exception_Mark;

   DMA_Error : exception;

private

   RX_Message : Message_From_Server with
     Volatile;

   TX_Message : Message_From_Client;

   procedure Set_TX_Message_Kind (Kind : Message_From_Client_Kind);
   procedure Set_TX_Message_CRC;
   procedure Transmit_TX_Message;

   Last_Step_Delta : Step_Delta;

end Server_Communication;
