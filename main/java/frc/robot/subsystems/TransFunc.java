package frc.robot.subsystems;

public class TransFunc {

    public float cof;
    public int j;
    
    public static float TransF(float[][] a, float number, int j , float cof) {
        float test = 0f;
        for (j = 0; j < a[0].length; j ++) {
            if (j < 1) {
                if (Math.abs(number) < a[0][j]){
                    if (number >= 0){
                        test = a[1][j];
                    }else{
                        test = -a[1][j];
                    }
                    break;
                }else{
                    continue;
                }
            } else {
                if (Math.abs(number) >= a[0][j-1] && Math.abs(number) <= a[0][j]) {
                    cof = (number / a[0][j]);
                    test =  a[1][j] * cof;
                    break;
                } else {
                    if (number >= 0){
                        test = a[1][j];
                    }else{
                        test = -a[1][j];
                    }
                }
            }
        }
        return test;
    }
}
