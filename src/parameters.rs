#![allow(non_upper_case_globals)]
#![allow(dead_code)]


    pub const InitializerNumMinFeatures: i32 = 100;
    pub const kInitializerNumMinTriangulatedPoints: i32 = 100;
    pub const kFeatureMatchRatioTestInitializer: f32 = 0.8;//   # ratio test used by Initializer  

    pub const kInitializerDesiredMedianDepth: i32 = 20 ;//   # when initializing, the initial median depth is computed and forced to this value (for better visualization is > 1) 
