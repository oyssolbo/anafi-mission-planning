(define (problem move_p0)
  (:domain move_domain)
  (:objects 
    d0 - drone 
    a1 a2 - location
  )
  (:init 
    (at d0 a1)
    (available a2)
    (path a1 a2)
    (path a2 a1)
  )
  (:goal
    (and 
      (at d0 a2) 
    )
  )
)