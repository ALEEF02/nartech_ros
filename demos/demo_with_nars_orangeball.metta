!(nartech.nars (Command volume=0))

; Reach the remembered location of an object by navigating to its coordinates.
!(nartech.nars (AddBeliefEternal ((((($location * $object) --> perceive) &/
                                   (({SELF} * $location) --> (^ go))) =/>
                                     (($location * ({SELF} & $object)) --> perceive))
                                       (1.0 0.9))))

; If we perceive ourselves colocated with the orange ball, pick it up.
!(nartech.nars (AddBeliefEternal ((((($location * ({SELF} & orangeball)) --> perceive)) =/>
                                   (({SELF} * orangeball) --> (^ pick)))
                                     (1.0 0.9))))

; If we perceive ourselves colocated with a person while carrying the orange ball, drop it.
!(nartech.nars (AddBeliefEternal ((((($location * ({SELF} & person)) --> perceive) &/
                                   (({SELF} * orangeball) --> carry)) =/>
                                     (^ drop))
                                       (1.0 0.9))))

(= (pickupgoal)
   (AddGoalEvent (((?1 * ({SELF} & orangeball)) --> perceive) (1.0 0.9))))

(= (dropgoal)
   (AddGoalEvent (((?1 * ({SELF} & person)) --> perceive) (1.0 0.9))))

(= (Step $stepTime $realTime)
   (if (== (nartech.ros.navigation) BUSY)
       (navigation stack busy)
       (let* (($objects  (nartech.ros.objects))
              ($unknowns (nartech.ros.objects.filter unknown $objects))
              ($armstate (nartech.ros.arm))
              ($goalcmd  (if (== $armstate FREE)
                             (pickupgoal)
                             (dropgoal))))
             (superpose
               ((nartech.nars.perceive $objects)
                (if (== $armstate FREE)
                    ()
                    (nartech.nars (AddBeliefEvent ((({SELF} * orangeball) --> carry) (1.0 0.9)))))
                (if (== () (nartech.nars.execute 5 $goalcmd))
                    (let (detection unknown $coordinates) (superpose $unknowns)
                         (nartech.ros (go $coordinates)))
                    ())
                (nartech.nars (Cycles 20)))))))
