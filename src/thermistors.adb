with STM32.GPIO;   use STM32.GPIO;
with STM32.Device; use STM32.Device;
with STM32.ADC;    use STM32.ADC;
with STM32.DMA;    use STM32.DMA;
with HAL;          use HAL;
with Heaters;

package body Thermistors is

   procedure Init is
   begin
      Enable_Clock (Thermistor_ADC);
      Enable_Clock (Thermistor_DMA_Controller);

      for C of Curves loop
         for P of C loop
            P := (Temp => 1_000_000.0 * celcius, Value => 0);
         end loop;
      end loop;

      Disable (Thermistor_ADC);

      Configure_Common_Properties
        (This           => Thermistor_ADC,
         Mode           => Independent,
         Prescaler      => Div_1,
         Clock_Mode     => PCLK2_Div_4,
         DMA_Mode       => Disabled,
         Sampling_Delay => Sampling_Delay_5_Cycles);

      Calibrate (Thermistor_ADC, Single_Ended);

      Configure_Unit (Thermistor_ADC, ADC_Resolution_12_Bits, Right_Aligned);
      Thermistor_ADC_Internal.CFGR2.ROVSE := True;   --  Regular oversampling
      Thermistor_ADC_Internal.CFGR2.OVSS  := 4;      --  4 bit shift
      Thermistor_ADC_Internal.CFGR2.OVSR  := 2#111#; --  256 samples

      Configure_Regular_Conversions
        (This        => Thermistor_ADC,
         Continuous  => False,
         Trigger     => Software_Triggered,
         Conversions => [for I in 1 .. Thermistor_Name'Pos (Thermistor_Name'Last) + 1 =>
           (Channel => Thermistor_ADC_Channels (Thermistor_Name'Val (I - 1)), Sample_Time => Sample_640P5_Cycles)]);

      Enable (Thermistor_ADC);

      for Thermistor in Thermistor_Name loop
         Configure_IO (Thermistor_GPIO_Points (Thermistor), (Mode => Mode_Analog, Resistors => Floating));
      end loop;

      Configure
        (Thermistor_DMA_Controller,
         Thermistor_DMA_Stream,
        (Channel                       => Thermistor_DMA_Channel,
          Direction                    => Peripheral_To_Memory,
          Increment_Peripheral_Address => False,
          Increment_Memory_Address     => True,
          Peripheral_Data_Format       => HalfWords,
          Memory_Data_Format           => HalfWords,
          Operation_Mode               => Normal_Mode,
          Priority                     => Thermistor_DMA_Priority,
          Memory_Burst_Size            => Memory_Burst_Single,
          Peripheral_Burst_Size        => Peripheral_Burst_Single));

      Enable_DMA (Thermistor_ADC);
   end Init;

   procedure Setup (Thermistor_Curves : Thermistor_Curves_Array; Heater_Map : Heater_Thermistor_Map) is
   begin
      for Thermistor in Thermistor_Name loop
         for I in Thermistor_Curve_Index loop
            Curves (Thermistor) (I) :=
              (Temp  => Dimensionless (Thermistor_Curves (Thermistor) (I).Temp) * celcius,
               Value => Thermistor_Curves (Thermistor) (I).Value);
         end loop;
      end loop;
      Heater_Thermistors := Heater_Map;
   end Setup;

   procedure Start_ISR_Loop is
   begin
      Start_Conversion;
   end Start_ISR_Loop;

   function Last_Reported_Temperature (Thermistor : Thermistor_Name) return Temperature is
   begin
      return Cached_Temperatures (Thermistor);
   end Last_Reported_Temperature;

   procedure Start_Conversion is
   begin
      Start_Transfer_with_Interrupts
        (This               => Thermistor_DMA_Controller,
         Stream             => Thermistor_DMA_Stream,
         Source             => Data_Register_Address (Thermistor_ADC),
         Destination        => ADC_Results'Address,
         Data_Count         => ADC_Results'Length,
         Enabled_Interrupts => [Transfer_Complete_Interrupt => True, others => False]);
      Start_Conversion (Thermistor_ADC);
   end Start_Conversion;

   function Interpolate (ADC_Val : ADC_Value; Thermistor : Thermistor_Name) return Temperature is
      Curve : Float_Thermistor_Curve renames Curves (Thermistor);
      Left  : Thermistor_Curve_Index := Thermistor_Curve'First;
      Right : Thermistor_Curve_Index := Thermistor_Curve'Last - 1;
      Mid   : Thermistor_Curve_Index;
   begin
      while Left <= Right loop
         Mid := Left + (Right - Left) / 2;
         if ADC_Val >= Curve (Mid).Value and then ADC_Val <= Curve (Mid + 1).Value then
            exit;
         elsif ADC_Val < Curve (Mid).Value then
            Right := Mid - 1;
         else
            Left := Mid + 1;
         end if;
      end loop;

      if Left > Right then
         --  Always return a high value to force the heater off as the ADC result should never go outside of the
         --  defined range under normal operation.
         return 1_000_000.0 * celcius;
      end if;

      declare
         Lower_Point : constant Float_Thermistor_Point := Curve (Mid);
         Upper_Point : constant Float_Thermistor_Point := Curve (Mid + 1);
      begin
         if Lower_Point.Temp = Upper_Point.Temp then
            return Lower_Point.Temp;
         else
            return
              Lower_Point.Temp +
              (Upper_Point.Temp - Lower_Point.Temp) / Dimensionless (Upper_Point.Value - Lower_Point.Value) *
                Dimensionless (ADC_Val - Lower_Point.Value);
         end if;
      end;
   end Interpolate;

   protected body ADC_Handler is
      procedure End_Of_Sequence_Handler is
         Temps : array (Thermistor_Name) of Temperature;
      begin
         Clear_All_Status (Thermistor_DMA_Controller, Thermistor_DMA_Stream);
         Start_Conversion;

         for Thermistor in Thermistor_Name loop
            declare
               Temp : Temperature := Interpolate (ADC_Results (Thermistor), Thermistor);
            begin
               Temps (Thermistor) := Temp;
            end;
         end loop;

         for Thermistor in Thermistor_Name loop
            Cached_Temperatures (Thermistor) := Temps (Thermistor);
         end loop;

         for Heater in Heater_Name loop
            Heaters.Update (Heater, Temps (Heater_Thermistors (Heater)));
         end loop;
      end End_Of_Sequence_Handler;
   end ADC_Handler;

end Thermistors;
