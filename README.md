# KSP-Auto-Landing
*Some python scripts enable your Kerbal rocket to land (safely, hopefully) at a fixed point*

---

### Usage:  
First, install the dependences:  
`pip install -r requirements.txt`  
  
Next, adjust settings in:  
`./Internal/params.py`(for control params)  
`./landing.py`(line 145, in function `create_target_reference_frame` ,`targets=`)
   
Finally, run the script:  
`python landing.py`

---

### References:
[G-FOLD-Python](https://github.com/jonnyhyman/G-FOLD-Python)  
[G-FOLD](https://github.com/Wrg1t/G-FOLD)  
[GFOLD_KSP](https://github.com/xdedss/GFOLD_KSP)  
[Lossless Convexification of Nonconvex Control Bound and Pointing Constraints of the Soft Landing Optimal Control Problem](http://www.larsblackmore.com/iee_tcst13.pdf)  
[ksp_simple_landing_mission](https://github.com/laishere/ksp_simple_landing_mission)

---

### Links:
[GFOLD-solver](https://github.com/ZCM8848/GFOLD-solver)

---

### Demo:
[Fixed-point landing terminal guidance of rockets based on GFOLD algorithm](https://www.bilibili.com/video/BV1M6gNe7ECU)  
[Demonstration of the whole process of the return-fixed-point landing algorithm of the New Glenn-like rocket](https://www.bilibili.com/video/BV1oXYDeLE3L/)  
[Use K.A.L. and chopsticks to catch a rocket](https://www.bilibili.com/video/BV1FKfSYFEAo/)