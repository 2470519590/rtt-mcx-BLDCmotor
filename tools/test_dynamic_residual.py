import unittest
import subprocess
from pathlib import Path
import numpy as np
from dynamic_residual_model import features, prepare


class DynamicTests(unittest.TestCase):
    def test_parity_requires_both_prediction_and_application(self):
        from verify_dynamic import verify
        model=dict(w1=np.zeros((8,3)),b1=np.zeros(8),w2=np.zeros(8),
                   b2=.01,linear=np.array([0,0,0,.02]))
        row=dict(nn_mode=7,clipped=0,fault=0,sensor_ok=1,calibrated=1,
                 target_speed_mrad_s=523,speed_mrad_s=0,nn_delta_uq_mv=10,
                 nn_alpha_milli=1000,uq_mv=300,base_uq_mv=200)
        self.assertTrue(verify([row],model,[0]*8+[1000])['passed'])
        self.assertFalse(verify([dict(row,uq_mv=310)],model,[0]*8+[1000])['passed'])
        self.assertTrue(verify([dict(row,nn_mode=8,uq_mv=310)],model,[0]*8+[1000])['passed'])
        self.assertFalse(verify([dict(row,nn_mode=8,uq_mv=310,nn_delta_uq_mv=99)],
                                model,[0]*8+[1000])['passed'])
    def test_actual_c_prediction_shadow_and_alpha_gate(self):
        from test_uart_rx_queue import wsl_path
        from dynamic_residual_model import predict
        root=Path(__file__).resolve().parents[1]
        binary=root/'debug'/'dynamic_residual_host'
        subprocess.run(['wsl.exe','--exec','gcc','-std=c99','-Wall','-Wextra','-Werror',
            '-fsanitize=address,undefined','-I'+wsl_path(root/'app'),
            wsl_path(root/'app/nn_controller.c'),wsl_path(root/'app/angle_shadow.c'),
            wsl_path(root/'tools/dynamic_residual_harness.c'),'-lm','-o',wsl_path(binary)],
            capture_output=True,check=True)
        model=dict(np.load(root/'data/dynamic_lookahead_linear_20261023.npz'))
        cases=[(mode,alpha,t,v) for mode in (7,8,9) for alpha in (0,500,1000)
               for t in (-1570,523) for v in (-2000,0,1700)]
        wire=''.join(f'{m} {a} {t/1000:.6f} {v/1000:.6f}\n' for m,a,t,v in cases)
        result=subprocess.run(['wsl.exe','--exec',wsl_path(binary)],input=wire,
                              text=True,capture_output=True,check=True)
        for case,line in zip(cases,result.stdout.splitlines()):
            mode,alpha,t,v=case
            x=features([t],[v])
            expected=(float(np.clip(np.r_[x[0],1]@model['linear'],-.03,.03)) if mode==9
                      else float(predict(model,x)[0]))
            delta,output=map(float,line.split())
            self.assertAlmostEqual(delta,expected,delta=2e-6)
            self.assertAlmostEqual(output,.3+(alpha/1000*expected if mode!=7 else 0),
                                   delta=3e-6)
        self.assertEqual(len(result.stdout.splitlines()),len(cases))
    def test_feature_clips_and_direction(self):
        x=features([1570,-1570,100000],[0,1570,-100000])
        np.testing.assert_array_equal(x[:,2],[0,1,1])
        self.assertEqual(x[2,0],2)
        self.assertEqual(x[2,1],-4)

    def test_horizon_never_crosses_a_command_boundary(self):
        rows=[]
        for step,base in ((0,100),(1,-100)):
            for i in range(61):
                rows.append(dict(config='friction',repeat=0,step=step,tick_ms=i*10,
                    target_speed_mrad_s=0,speed_mrad_s=0,speed_kp_milli=150,
                    base_uq_mv=base+i,fault=0,clipped=0,sensor_ok=1,calibrated=1))
        _,y,_=prepare(rows)
        np.testing.assert_allclose(y,.025,atol=1e-7)
        self.assertEqual(len(y),72)


if __name__ == '__main__':
    unittest.main()
