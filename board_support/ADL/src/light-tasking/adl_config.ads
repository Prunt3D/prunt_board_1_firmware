--  This package was generated by the Ada_Drivers_Library project wizard script
package ADL_Config is
   Board                          : constant String  := "PRUNT_BOARD_1";     -- From user input
   Architecture                   : constant String  := "ARM";               -- From board definition
   Vendor                         : constant String  := "STMicro";           -- From board definition
   Device_Family                  : constant String  := "STM32G4";           -- From board definition
   Device_Name                    : constant String  := "STM32G474xx";       -- From board definition
   CPU_Core                       : constant String  := "ARM Cortex-M4F";    -- From mcu definition
   High_Speed_External_Clock      : constant         := 16_000_000;          -- From board definition
   Number_Of_Interrupts           : constant         := 102;                 -- From MCU definition
   Has_ZFP_Runtime                : constant String  := "False";             -- From board definition
   Has_Ravenscar_SFP_Runtime      : constant String  := "True";              -- From board definition
   Has_Ravenscar_Full_Runtime     : constant String  := "True";              -- From board definition
   Runtime_Profile                : constant String  := "sfp";               -- From user input
   Runtime_Name_Suffix            : constant String  := "stm32g474";         -- From board definition
   Runtime_Name                   : constant String  := "sfp-stm32g474";     -- From user input
   Use_Startup_Gen                : constant Boolean := False;               -- From user input
   Has_Custom_Memory_Area_1       : constant Boolean := False;               -- From user input
   Boot_Memory                    : constant String  := "flash";             -- From user input
   Max_Path_Length                : constant         := 1024;                -- From user input
   Max_Mount_Points               : constant         := 2;                   -- From user input
   Max_Mount_Name_Length          : constant         := 128;                 -- From user input
end ADL_Config;
