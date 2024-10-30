def density(altitude):
    d = (-6.06523518e-24 * altitude**5 + 
         1.45080167e-18 * altitude**4 +
         -1.34865289e-13 * altitude**3 +
         6.07676141e-09 * altitude**2 + 
         -1.32638898e-04 * altitude + 
         1.12566181e+00)
    return d