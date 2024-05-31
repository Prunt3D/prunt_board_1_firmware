with Messages; use Messages;
private with Hardware_Configuration;

package Thermistors is

   subtype Celcius is Float;

   procedure Init;
   procedure Setup (Thermistor_Curves : Thermistor_Curves_Array; Heater_Map : Heater_Thermistor_Map);
   procedure Start_ISR_Loop;
   function Last_Reported_Temperature (Thermistor : Thermistor_Name) return Celcius;

private

   use Hardware_Configuration;

   type ADC_Results_Type is array (Thermistor_Name) of ADC_Value with
     Alignment => 2, Pack, Volatile, Volatile_Components;
   ADC_Results : aliased ADC_Results_Type;

   type Float_Reported_Temperatures_Atomic is array (Thermistor_Name) of Celcius with
     Atomic_Components, Volatile, Volatile_Components;
   Cached_Temperatures : Float_Reported_Temperatures_Atomic := (others => 1_000_000.0);

   type Float_Thermistor_Point is record
      Temp  : Celcius;
      Value : ADC_Value;
   end record;

   type Float_Thermistor_Curve is array (Thermistor_Curve_Index) of Float_Thermistor_Point with
     Volatile_Components;

   type Float_Thermistor_Curves_Array is array (Thermistor_Name) of Float_Thermistor_Curve with
     Volatile_Components;

   Curves : Float_Thermistor_Curves_Array := [others => [others => (Temp => 1_000_000.0, Value => 0)]] with
     Volatile, Linker_Section => (".ccmbss");

   Heater_Thermistors : Heater_Thermistor_Map with
     Volatile;

   procedure Start_Conversion;
   function Interpolate (ADC_Val : ADC_Value; Thermistor : Thermistor_Name) return Celcius;

   protected ADC_Handler is
      pragma Interrupt_Priority (Thermistor_DMA_Interrupt_Priority);
   private
      procedure End_Of_Sequence_Handler with
        Attach_Handler => Thermistor_DMA_Interrupt_ID;
   end ADC_Handler;

end Thermistors;
