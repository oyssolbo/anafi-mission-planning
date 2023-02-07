(define (problem task)
  (:domain drone_sar_fuel)
  (:objects
      wp0 wp1 wp2 wp3 wp4 wp5 wp6 hp1 hp2 sa1 sa2 sa3 - waypoint
      drone - robot
  )
  (:init
      (robot_at drone wp0)

      (undocked drone)

      (charge_at wp1)
      (charge_at wp2)

      (= (state_of_charge drone) 96)

  )
  
  (:goal (and
      (photographed wp3)
      (photographed wp4)
      (photographed wp5)
    )
  )
)
