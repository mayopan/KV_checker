# KV_checker
    KV checker for R/C
    
    ![frontside](https://user-images.githubusercontent.com/49590865/160167926-fd5ecf9b-bb03-4780-8812-950123f6ed96.jpg)

    
    This program is for the original KV checker board using Seeed XIAO.
    Schematic of the board is shown in github -> https://github.com/mayopan/KV_checker

    It counts rotation of tire using photo reflector. You have to put some light color musking tape on the tire.
    It also measures voltage of battery with adc.
    Parameters listed below is modified by Setting mode with UP/DOWN/SELECT buttons.

      *Pinion Gear Teeth number
      *Spar Gear Teeth number
      .1st Gear ratio (calculated with the two gear's number. Not set manually)
      *2nd Gear ratio
      *Tire dia

    KV is calculated from tire rotation speed with gear ratio parameter.
    So you have to set the gear ratio parameter correctly to get true motor KV.
    KVT is easier to check the equality of the top speed performance per voltage that includes motor KV and gear ratio.
    For example, if the race regulation says KV < 2800rpm/V and gear ratio > 7,
    KVT < 2800 / 7 = 400.

    It has 6 mode for measure listed below.

      *KV       [rpm/V]     ...Rotation speed of Motor / Battery voltage
      *KVT      [rpm/V]     ...Rotation speed of Tire / Battery voltage
      *VOLTAGE  [V]         ...Battery voltage
      *SPD      [km/h]      ...Speed
      *TIRE RPM [rpm]       ...Rotation speed of tire
      *MOTOR RPM[rpm]       ...Rotation speed of motor
