#!/usr/bin/env python3
"""Portable regression for the native DPI test's layout-observation barrier."""
import copy
import ast
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest

spec = importlib.util.spec_from_file_location('diagnostic_geometry', Path(__file__).with_name('diagnostic-geometry.py'))
geometry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(geometry)


class LayoutObservation(unittest.TestCase):
    def setUp(self):
        # Native 6160d3e4 150% failure capture: all current controls/rail fit.
        self.expected = {'Menu': (1166, 51, 1262, 123),
                         'Navigation': (17, 711, 185, 783),
                         'System': (1094, 711, 1262, 783)}
        self.record = {'runtime': {'ui_update': {'ticks': '12'}, 'display': {
            'interaction_controls': [dict(label=label, x=r[0], y=r[1],
                                          width=r[2]-r[0], height=r[3]-r[1],
                                          visible=True, enabled=True)
                                     for label, r in self.expected.items()],
            'rail_regions': [{'label': 'sog', 'x': 1065, 'y': 129, 'width': 204,
                              'height': 143, 'visible': True}]}}}

    def test_exact_current_native_geometry_after_publication_passes(self):
        self.assertTrue(geometry.matches_native_controls(self.record, self.expected, 8))

    def test_even_matching_older_snapshot_waits_for_new_publication(self):
        self.assertFalse(geometry.matches_native_controls(self.record, self.expected, 12))
        self.assertFalse(geometry.matches_native_controls(self.record, self.expected, 13))

    def test_newer_tick_with_old_frame_position_still_refuses(self):
        old = copy.deepcopy(self.record)
        old['runtime']['display']['interaction_controls'][0]['x'] += 64
        self.assertFalse(geometry.matches_native_controls(old, self.expected, 8))
        self.assertTrue(geometry.matches_native_controls(self.record, self.expected, 8))

    def test_wrong_size_or_vertical_layout_is_not_a_current_observation(self):
        for field in ('x', 'y', 'width', 'height'):
            with self.subTest(field=field):
                changed = copy.deepcopy(self.record)
                changed['runtime']['display']['interaction_controls'][1][field] += 1
                self.assertFalse(geometry.matches_native_controls(changed, self.expected, 8))

    def test_missing_duplicate_and_hidden_chrome_refuse(self):
        for kind in ('missing', 'duplicate', 'hidden'):
            with self.subTest(kind=kind):
                changed = copy.deepcopy(self.record)
                controls = changed['runtime']['display']['interaction_controls']
                if kind == 'missing':
                    controls.pop()
                elif kind == 'duplicate':
                    controls.append(copy.deepcopy(controls[0]))
                else:
                    controls[0]['visible'] = False
                self.assertFalse(geometry.matches_native_controls(changed, self.expected, 8))

    def test_missing_or_malformed_observation_refuses(self):
        self.assertFalse(geometry.matches_native_controls({}, self.expected, 8))
        self.record['runtime']['ui_update']['ticks'] = 'invalid'
        self.assertFalse(geometry.matches_native_controls(self.record, self.expected, 8))

    def test_empty_native_reference_cannot_pass_vacuously(self):
        self.assertFalse(geometry.matches_native_controls(self.record, {}, 8))

    def test_scrolled_row_pairs_offscreen_bounds_without_asserting_visibility(self):
        row=self.record['runtime']['display']['interaction_controls'][0]
        row['visible']=False
        self.assertFalse(geometry.matches_native_controls(self.record,self.expected,8))
        self.assertTrue(geometry.matches_native_controls(self.record,self.expected,8,require_visible=False))
        row['y']+=24
        self.assertFalse(geometry.matches_native_controls(self.record,self.expected,8,require_visible=False))

    def test_offscreen_observation_still_rejects_old_ticks_and_duplicate_rows(self):
        self.assertFalse(geometry.matches_native_controls(self.record,self.expected,12,require_visible=False))
        controls=self.record['runtime']['display']['interaction_controls']
        controls.append(copy.deepcopy(controls[0]))
        self.assertFalse(geometry.matches_native_controls(self.record,self.expected,8,require_visible=False))

    def test_real_rail_clipping_is_not_filtered_by_observation_barrier(self):
        # The exact rejected 6160 rail sample must reach the unchanged bounds
        # assertion if it recurs with CURRENT chrome. Do not poll until it fits.
        region = self.record['runtime']['display']['rail_regions'][0]
        region.update(x=1129, width=204, height=122)
        self.assertTrue(geometry.matches_native_controls(self.record, self.expected, 8))
        with self.assertRaises(AssertionError):
            assert 0 <= region['x'] < region['x']+region['width'] <= 1280


class MainButtonGeometry(unittest.TestCase):
    """Exercise the actual gate against immutable-HTML desktop measurements.

    Native 488fbbdf fullscreen was correctly 69px at 1920x1080; the old
    height-only oracle rejected it as 61px. These fakes check oracle selection
    and refusal behavior, not native rendering, DPI or touch acceptance.
    """
    def check_buttons(self, width, height, scale, nav, pilot=87, mutate=None):
        source=Path(__file__).with_name('smoke-dpi-windows.py')
        tree=ast.parse(source.read_text())
        function=next(n for n in tree.body if isinstance(n,ast.FunctionDef)
                      and n.name=='main_buttons')
        factor=scale/100
        controls=[dict(label=label,x=9*factor,y=90*factor,width=69*factor,
                       height=nav*factor,visible=True,enabled=True)
                  for label in ('Chart','Passage','Traffic','Energy','Instruments',
                                'Anchor','Radar','Settings')]
        controls += [dict(label=label,x=100*factor,y=90*factor,width=44*factor,
                          height=h*factor,visible=True,enabled=True)
                     for label,h in [('Autopilot',pilot),('Day',44),('+',44),
                                     ('−',44),('Follow boat',44)]]
        footer=dict(x=0,y=height-34*factor,width=width,height=34*factor)
        controls.append(dict(label='Source health',x=0,y=footer['y'],
                             width=44*factor,height=34*factor,visible=True,enabled=True))
        if mutate:mutate(controls)
        display=dict(light='Day',interaction_controls=controls,footer_region=footer,
                     footer_middle_visible=width/factor>1100)
        def rect():return SimpleNamespace(left=0,top=0,right=width,bottom=height)
        ui=SimpleNamespace(W=SimpleNamespace(RECT=rect),GetWindowRect=lambda *_:True,
                           GetClientRect=lambda *_:True,
                           children=lambda _:[(2,'OpenNav status footer')])
        namespace=dict(ui=ui,C=SimpleNamespace(byref=lambda r:r),handle=1,
                       current_layout_observation=lambda:{'runtime':{'display':display}},
                       bounds=lambda _:SimpleNamespace(left=0,top=footer['y'],
                                                       right=width,bottom=height))
        exec(compile(ast.Module(body=[function],type_ignores=[]),str(source),'exec'),namespace)
        return namespace['main_buttons'](scale)

    def test_html_width_boundary_and_compact_cascade(self):
        # Measured from unchanged HTML at DPR1; includes both sides of each rule.
        for width,height,nav,pilot in [(1280,800,61,87),(1499,1080,61,87),
                (1500,1080,69,87),(1920,1080,69,87),(1920,741,69,87),
                (1920,740,51,74),(1920,601,51,74),(1920,600,43,66)]:
            with self.subTest(width=width,height=height):
                self.check_buttons(width,height,100,nav,pilot)

    def test_width_breakpoint_uses_logical_client_pixels_at_actual_scale(self):
        for width,height,scale,nav,pilot in [(1920,1080,125,69,87),
                (1920,1080,150,51,74),(1874,1000,125,61,87),
                (1875,1000,125,69,87),(2250,1200,150,69,87)]:
            # 1920 / 1.25 = 1536; 1920 / 1.5 = 1280, height=720.
            with self.subTest(width=width,scale=scale):
                self.check_buttons(width,height,scale,nav,pilot)

    def test_wrong_geometry_still_fails_exact_gate(self):
        for width,nav in [(1920,61),(1499,69),(1920,71)]:
            with self.subTest(width=width,nav=nav),self.assertRaises(AssertionError):
                self.check_buttons(width,1080,100,nav)

    def test_clipped_control_still_fails(self):
        with self.assertRaises(AssertionError):
            self.check_buttons(1920,1080,100,69,
                               mutate=lambda controls:controls[0].update(x=-1))


class PreferencesTouch(unittest.TestCase):
    """Exercise the actual Windows harness function with native observations.

    These fakes test the evidence gate, not Windows touch/render acceptance.
    In particular, a clipped but WS_VISIBLE HWND must never receive a tap.
    """
    def setUp(self):
        source=Path(__file__).with_name('smoke-dpi-windows.py')
        tree=ast.parse(source.read_text())
        function=next(n for n in tree.body if isinstance(n,ast.FunctionDef)
                      and n.name=='touch_preferences_action')
        self.rects=[(120,410,480,482),(120,220,480,292)]
        self.index=0;self.pans=[];self.taps=[]
        self.enabled=True;self.visible_override=None
        self.covered=False;self.pan_overlay=False
        def bounds(handle):
            r=self.rects[self.index] if handle==22 else (100,100,500,400)
            return SimpleNamespace(**dict(zip(('left','top','right','bottom'),r)))
        def observation(label):
            r=self.rects[self.index]
            visible=100<=r[1]<r[3]<=400 if self.visible_override is None else self.visible_override
            control=dict(label=label,x=r[0],y=r[1],width=r[2]-r[0],height=r[3]-r[1],
                         visible=visible,enabled=self.enabled)
            return {'runtime':{'ui_update':{'ticks':self.index+1},'display':{
                'interaction_controls':[control]}}},22
        def hit(point):
            r=self.rects[self.index]
            if r[0]<=point.x<r[2] and r[1]<=point.y<r[3]:
                return 99 if self.covered else 22
            return 99 if self.pan_overlay else 21
        def inject(kind,*coordinates):
            if kind=='--pan':
                self.pans.append(coordinates)
                self.index=min(self.index+1,len(self.rects)-1)
            elif kind=='--tap':self.taps.append(coordinates)
            else:raise AssertionError('Unexpected input')
            return {'touch_injected':True}
        ui=SimpleNamespace(wait_window=lambda *args:(20,7),user=None,
            W=SimpleNamespace(HWND=int,POINT=lambda x,y:SimpleNamespace(x=x,y=y)),
            declare=lambda *args:lambda:20,SetForegroundWindow=lambda _:True,
            IsWindowEnabled=lambda _:self.enabled,GetParent=lambda h:{22:21,21:20}[h],
            WindowFromPoint=hit,IsChild=lambda parent,child:parent==21 and child==22)
        namespace=dict(ui=ui,pid=7,bounds=bounds,preferences_observation=observation,
                       dpi=inject,time=SimpleNamespace(sleep=lambda _:None))
        exec(compile(ast.Module(body=[function],type_ignores=[]),str(source),'exec'),namespace)
        self.action=namespace['touch_preferences_action']

    def reject(self):
        with self.assertRaises(AssertionError):self.action('Battery & reserve',125)
        self.assertEqual(self.taps,[],'Invalid/clipped action received a tap')

    def test_clipped_action_is_scrolled_before_exact_native_tap(self):
        result=self.action('Battery & reserve',125)
        self.assertEqual(len(self.pans),1)
        self.assertEqual(self.taps,[(300,256)])
        self.assertEqual([r['visible'] for r in result['observations']],[False,True])
        self.assertTrue(result['native_hit_target_verified'])

    def test_visible_action_does_not_scroll(self):
        self.rects=[(120,220,480,292)]
        self.action('Battery & reserve',100)
        self.assertEqual(self.pans,[])
        self.assertEqual(self.taps,[(300,256)])

    def test_no_scroll_progress_refuses(self):
        self.rects=self.rects[:1]
        self.reject()
        self.assertEqual(len(self.pans),1)

    def test_scroll_attempts_are_bounded(self):
        self.rects=[(120,5000-i,480,5072-i) for i in range(41)]
        self.reject()
        self.assertEqual(len(self.pans),40)

    def test_diagnostic_visible_flag_cannot_hide_native_clipping(self):
        self.visible_override=True
        self.reject()
        self.assertEqual(self.pans,[])

    def test_horizontal_clipping_refuses(self):
        self.rects=[(90,220,480,292)]
        self.reject()

    def test_disabled_action_refuses(self):
        self.enabled=False
        self.reject()

    def test_covering_surface_prevents_tap(self):
        self.rects=[(120,220,480,292)];self.covered=True
        self.reject()

    def test_pan_cannot_target_another_surface(self):
        self.pan_overlay=True
        self.reject()
        self.assertEqual(self.pans,[])


if __name__ == '__main__':
    unittest.main()
