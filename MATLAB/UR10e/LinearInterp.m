function v = LinearInterp(v0, v1, t)
    v = (1 - t) * v0 + t * v1; 
end