(define (problem move_p0)
  (:domain move_domain)
  (:objects 
    d0 - drone
    a1 a2 a3 - search_area
    l1 l2 - loiter_area
    h1 h2 - helipad
    bat - battery
    mar - marker
  )
  (:init 
    (at d0 h1)
    (landed d0)
    (has d0 bat)
    (has d0 mar)

    (available a1)
    (available a2)
    (available a3)
    (available l1)
    (available l2)
    (available h2)

    (path a1 a2)
    (path a2 a1)
    (path a1 a3)
    (path a3 a1)
    (path a2 a3)
    (path a3 a2)

    (path l1 a1)
    (path a1 l1)
    (path l1 a2)
    (path a2 l1)

    (path l2 a1)
    (path a1 l2)
    (path l2 a2)
    (path a2 l2)

    (path l1 h1)
    (path h1 l1)
    (path l2 h2)
    (path h2 l2)

    (= (distance h1 l1)   5   )
    (= (distance l1 h1)   5   )
    (= (distance l1 a1)   100 )
    (= (distance a1 l1)   100 )
    (= (distance a1 a2)   50  )
    (= (distance a2 a1)   50  )
    (= (distance a1 a3)   50  )
    (= (distance a3 a1)   50  )
    (= (distance a3 a2)   150 )
    (= (distance a2 a3)   150 )
    (= (distance h2 l2)   25  )
    (= (distance l2 h2)   25  )
    (= (distance l2 a2)   75  )
    (= (distance a2 l2)   75  )

    (= (velocity d0) 5 )


  )
  (:goal
    (and 
      (at d0 h2) 
      (searched a1)
      (searched a2)
      (searched a3)
      ; (preference (searched a1))
      ; (preference (searched a2))
      ; (preference (searched a3))
      (landed d0)
    )
  )
  ; Solves this problem in 27.91 ms
  ; which is far more efficient than the similar problem solved by the
  ; Graphplan algorithm developed by Hinostroza 
)