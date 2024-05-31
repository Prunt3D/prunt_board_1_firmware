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
      1   => 58_558,
      2   => 29_279,
      3   => 19_520,
      4   => 14_640,
      5   => 11_712,
      6   => 9_760,
      7   => 8_366,
      8   => 7_320,
      9   => 6_507,
      10  => 5_856,
      11  => 5_324,
      12  => 4_880,
      13  => 4_505,
      14  => 4_183,
      15  => 3_904,
      16  => 3_660,
      17  => 3_445,
      18  => 3_254,
      19  => 3_082,
      20  => 2_928,
      21  => 2_789,
      22  => 2_662,
      23  => 2_546,
      24  => 2_440,
      25  => 2_343,
      26  => 2_253,
      27  => 2_169,
      28  => 2_092,
      29  => 2_020,
      30  => 1_952,
      31  => 1_889,
      32  => 1_830,
      33  => 1_775,
      34  => 1_723,
      35  => 1_674,
      36  => 1_627,
      37  => 1_583,
      38  => 1_541,
      39  => 1_502,
      40  => 1_464,
      41  => 1_429,
      42  => 1_395,
      43  => 1_362,
      44  => 1_331,
      45  => 1_302,
      46  => 1_273,
      47  => 1_246,
      48  => 1_220,
      49  => 1_196,
      50  => 1_172,
      51  => 1_149,
      52  => 1_127,
      53  => 1_105,
      54  => 1_085,
      55  => 1_065,
      56  => 1_046,
      57  => 1_028,
      58  => 1_010,
      59  => 993,
      60  => 976,
      61  => 960,
      62  => 945,
      63  => 930,
      64  => 915,
      65  => 901,
      66  => 888,
      67  => 874,
      68  => 862,
      69  => 849,
      70  => 837,
      71  => 825,
      72  => 814,
      73  => 803,
      74  => 792,
      75  => 781,
      76  => 771,
      77  => 761,
      78  => 751,
      79  => 742,
      80  => 732,
      81  => 723,
      82  => 715,
      83  => 706,
      84  => 698,
      85  => 689,
      86  => 681,
      87  => 674,
      88  => 666,
      89  => 658,
      90  => 651,
      91  => 644,
      92  => 637,
      93  => 630,
      94  => 623,
      95  => 617,
      96  => 610,
      97  => 604,
      98  => 598,
      99  => 592,
      100 => 586,
      101 => 580,
      102 => 575,
      103 => 569,
      104 => 564,
      105 => 558,
      106 => 553,
      107 => 548,
      108 => 543,
      109 => 538,
      110 => 533,
      111 => 528,
      112 => 523,
      113 => 519,
      114 => 514,
      115 => 510,
      116 => 505,
      117 => 501,
      118 => 497,
      119 => 493,
      120 => 488,
      121 => 484,
      122 => 480,
      123 => 477,
      124 => 473,
      125 => 469,
      126 => 465,
      127 => 462,
      128 => 458,
      129 => 454,
      130 => 451,
      131 => 448,
      132 => 444,
      133 => 441,
      134 => 437,
      135 => 434,
      136 => 431,
      137 => 428,
      138 => 425,
      139 => 422,
      140 => 419,
      141 => 416,
      142 => 413,
      143 => 410,
      144 => 407,
      145 => 404,
      146 => 402,
      147 => 399,
      148 => 396,
      149 => 394,
      150 => 391,
      151 => 388,
      152 => 386,
      153 => 383,
      154 => 381,
      155 => 378,
      156 => 376,
      157 => 373,
      158 => 371,
      159 => 369,
      160 => 366,
      161 => 364,
      162 => 362,
      163 => 360,
      164 => 358,
      165 => 355,
      166 => 353,
      167 => 351,
      168 => 349,
      169 => 347,
      170 => 345,
      171 => 343,
      172 => 341,
      173 => 339,
      174 => 337,
      175 => 335,
      176 => 333,
      177 => 331,
      178 => 329,
      179 => 328,
      180 => 326,
      181 => 324,
      182 => 322,
      183 => 320,
      184 => 319,
      185 => 317,
      186 => 315,
      187 => 314,
      188 => 312,
      189 => 310,
      190 => 309,
      191 => 307,
      192 => 305,
      193 => 304,
      194 => 302,
      195 => 301,
      196 => 299,
      197 => 298,
      198 => 296,
      199 => 295,
      200 => 293);

end Step_Generator;
