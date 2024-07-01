private with Ada.Interrupts.Names;
private with System;
private with HAL;
with Messages;               use Messages;
with Hardware_Configuration; use Hardware_Configuration;

package Step_Generator is

   procedure Init;

   procedure Enqueue (Steps : Step_Delta);
   procedure Setup_Loop (Input_Switch : Input_Switch_Name; Until_State : Input_Switch_State);
   procedure Enqueue_Start_Loop;
   procedure Enqueue_Stop_Loop;
   procedure Wait_Until_Idle;
   procedure Force_Start;

   Empty_Buffer_Error : exception;

private

   use HAL;

   type Step_Delta_Buffer_Index is mod 2**13;

   Step_Delta_Buffer : array (Step_Delta_Buffer_Index) of Step_Delta with
     Volatile_Components;

   Step_Delta_Buffer_Writer_Index : Step_Delta_Buffer_Index := 0 with
     Volatile, Atomic;
   Step_Delta_Buffer_Reader_Index : Step_Delta_Buffer_Index := 0 with
     Volatile, Atomic;

   Step_Delta_Buffer_Loop_Start_Index : Step_Delta_Buffer_Index := 0 with
     Volatile, Atomic;
   Step_Delta_Buffer_Loop_End_Index   : Step_Delta_Buffer_Index := 0 with
     Volatile, Atomic;

   Step_Delta_Buffer_Loop_Enabled : Boolean := False with
     Volatile, Atomic;

   Loop_Until_State  : Input_Switch_State with
     Volatile;
   Loop_Input_Switch : Input_Switch_Name with
     Volatile;

   Is_Idle : Boolean := True with
     Volatile, Atomic;

   Buffer_Ran_Dry : Boolean := False with
     Volatile, Atomic;

   protected Timer_Reload_Handler is
      pragma Interrupt_Priority (Step_Generator_Interrupt_Priority);
   private
      procedure Master_Update_Handler with
        Attach_Handler => Ada.Interrupts.Names.HRTIM_Master_IRQn_Interrupt;
   end Timer_Reload_Handler;

   Step_Count_To_Period : constant array (Step_Count) of UInt16 :=
     (0   => 100,
      1   => 64_622,
      2   => 32_312,
      3   => 21_541,
      4   => 16_156,
      5   => 12_925,
      6   => 10_771,
      7   => 9_232,
      8   => 8_078,
      9   => 7_180,
      10  => 6_462,
      11  => 5_875,
      12  => 5_385,
      13  => 4_971,
      14  => 4_616,
      15  => 4_308,
      16  => 4_039,
      17  => 3_801,
      18  => 3_590,
      19  => 3_401,
      20  => 3_231,
      21  => 3_077,
      22  => 2_937,
      23  => 2_810,
      24  => 2_693,
      25  => 2_585,
      26  => 2_486,
      27  => 2_393,
      28  => 2_308,
      29  => 2_228,
      30  => 2_154,
      31  => 2_085,
      32  => 2_019,
      33  => 1_958,
      34  => 1_901,
      35  => 1_846,
      36  => 1_795,
      37  => 1_747,
      38  => 1_701,
      39  => 1_657,
      40  => 1_616,
      41  => 1_576,
      42  => 1_539,
      43  => 1_503,
      44  => 1_469,
      45  => 1_436,
      46  => 1_405,
      47  => 1_375,
      48  => 1_346,
      49  => 1_319,
      50  => 1_292,
      51  => 1_267,
      52  => 1_243,
      53  => 1_219,
      54  => 1_197,
      55  => 1_175,
      56  => 1_154,
      57  => 1_134,
      58  => 1_114,
      59  => 1_095,
      60  => 1_077,
      61  => 1_059,
      62  => 1_042,
      63  => 1_026,
      64  => 1_010,
      65  => 994,
      66  => 979,
      67  => 965,
      68  => 950,
      69  => 937,
      70  => 923,
      71  => 910,
      72  => 898,
      73  => 885,
      74  => 873,
      75  => 862,
      76  => 850,
      77  => 839,
      78  => 829,
      79  => 818,
      80  => 808,
      81  => 798,
      82  => 788,
      83  => 779,
      84  => 769,
      85  => 760,
      86  => 751,
      87  => 743,
      88  => 734,
      89  => 726,
      90  => 718,
      91  => 710,
      92  => 702,
      93  => 695,
      94  => 687,
      95  => 680,
      96  => 673,
      97  => 666,
      98  => 659,
      99  => 653,
      100 => 646,
      101 => 640,
      102 => 634,
      103 => 627,
      104 => 621,
      105 => 615,
      106 => 610,
      107 => 604,
      108 => 598,
      109 => 593,
      110 => 587,
      111 => 582,
      112 => 577,
      113 => 572,
      114 => 567,
      115 => 562,
      116 => 557,
      117 => 552,
      118 => 548,
      119 => 543,
      120 => 539,
      121 => 534,
      122 => 530,
      123 => 525,
      124 => 521,
      125 => 517,
      126 => 513,
      127 => 509,
      128 => 505,
      129 => 501,
      130 => 497,
      131 => 493,
      132 => 490,
      133 => 486,
      134 => 482,
      135 => 479,
      136 => 475,
      137 => 472,
      138 => 468,
      139 => 465,
      140 => 462,
      141 => 458,
      142 => 455,
      143 => 452,
      144 => 449,
      145 => 446,
      146 => 443,
      147 => 440,
      148 => 437,
      149 => 434,
      150 => 431,
      151 => 428,
      152 => 425,
      153 => 422,
      154 => 420,
      155 => 417,
      156 => 414,
      157 => 412,
      158 => 409,
      159 => 406,
      160 => 404,
      161 => 401,
      162 => 399,
      163 => 396,
      164 => 394,
      165 => 392,
      166 => 389,
      167 => 387,
      168 => 385,
      169 => 382,
      170 => 380,
      171 => 378,
      172 => 376,
      173 => 374,
      174 => 371,
      175 => 369,
      176 => 367,
      177 => 365,
      178 => 363,
      179 => 361,
      180 => 359,
      181 => 357,
      182 => 355,
      183 => 353,
      184 => 351,
      185 => 349,
      186 => 347,
      187 => 346,
      188 => 344,
      189 => 342,
      190 => 340,
      191 => 338,
      192 => 337,
      193 => 335,
      194 => 333,
      195 => 331,
      196 => 330,
      197 => 328,
      198 => 326,
      199 => 325,
      200 => 323);

end Step_Generator;
