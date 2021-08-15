(define (problem prob1)
(:domain localization)
(:objects
     r0 r1 r2 r3 r4 - region
     R2D2 - robot
)
(:init
    (robot_in R2D2 r0)

    (= (act-cost) 0)
    (= (dummy) 0)

    (connected r0 r1) (connected r1 r0)
    (connected r0 r2) (connected r2 r0)
    (connected r0 r3) (connected r3 r0)
    (connected r0 r4) (connected r4 r0)

    (connected r1 r2) (connected r2 r1)
    (connected r1 r4) (connected r4 r1)
    (connected r1 r3) (connected r3 r1)

    (connected r2 r3) (connected r3 r2)
    (connected r2 r4) (connected r4 r2)
    (connected r3 r4) (connected r4 r3)
 )

(:goal 
     (and (visited r1)  (visited r2) 
          (visited r3) (visited r4))
)

     (:metric minimize (act-cost))
)


