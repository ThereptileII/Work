"""Shared native Preferences touch proof; importing does not launch or mutate UI."""
import ctypes as C
import time

def vessel_form_contract(record,require_save_visible=False):
    controls=record['runtime']['display']['interaction_controls']
    fields=['Field: Vessel name','Field: Draft · metres','Field: Safety depth · metres',
            'Field: Usable battery capacity · kWh','Field: Minimum reserve · %']
    observed=[c['label'] for c in controls if c['label'].startswith('Field: ')]
    assert sorted(observed)==sorted(fields),('Vessel form fields changed',observed)
    save=[c for c in controls if c['label']=='Save vessel profile']
    assert len(save)==1 and save[0]['enabled'],'Vessel profile Save identity must remain available'
    if require_save_visible:
        assert save[0]['visible'],'Save action must be fully visible and reachable without activation'
    return {'fields':fields,'save':{'label':save[0]['label'],'enabled':save[0]['enabled'],
                                    'visible':save[0]['visible']}}

def preferences_pan_path(viewport,target,scale,is_body):
    """Choose a vertical body gutter; never start a scroll on a native field.

    Rectangles are physical screen (left, top, right, bottom). The caller must
    supply live native hit testing; geometry alone cannot establish a recipient.
    """
    left,top,right,bottom=viewport
    padding=round(24*scale/100);distance=round(160*scale/100)
    gutter=round(8*scale/100)
    assert left<right and bottom-top>2*padding,'No usable Preferences pan area'
    below=target[3]>bottom
    assert below or target[1]<top,'Preferences action needs no pan'
    start=bottom-padding if below else top+padding
    end=max(top+padding,start-distance) if below else min(bottom-padding,start+distance)
    assert start!=end,'No usable Preferences pan distance'
    for x in (left+gutter,right-gutter):
        if left<x<right and is_body(x,start) and is_body(x,end):
            return (x,start),(x,end)
    raise AssertionError('No unobscured Preferences body gutter for native touch')

def reach_preferences_action(label,scale,*,ui,pid,observe,bounds,dpi,report):
    """Reach an action by body-targeted native touch, without activating it."""
    popup,_=ui.wait_window('OpenNav preferences',pid)
    foreground=ui.declare(ui.user,'GetForegroundWindow',ui.W.HWND)
    ui.SetForegroundWindow(popup)
    attempts=[];last_rect=None
    report.setdefault('preferences_scroll_attempts',[]).append(
        {'label':label,'scale':scale,'attempts':attempts})
    for _ in range(40):
        assert foreground()==popup,'Another window interrupted the Preferences gesture'
        observed,target=observe(label)
        controls=observed['runtime']['display']['interaction_controls']
        found=[c for c in controls if c['label']==label]
        assert len(found)==1,(label,'Missing or duplicate paired action')
        control=found[0]
        assert control['enabled'] and ui.IsWindowEnabled(target),(label,'Preferences action disabled')
        rect=bounds(target);native=(rect.left,rect.top,rect.right,rect.bottom)
        assert native==(control['x'],control['y'],control['x']+control['width'],control['y']+control['height']), 'Preferences moved after its observation'
        body=ui.GetParent(target)
        assert ui.GetParent(body)==popup,'Preferences target is not in this drawer body'
        viewport=bounds(body)
        assert viewport.left<=rect.left<rect.right<=viewport.right,(label,'Action horizontally clipped')
        visible=viewport.top<=rect.top<rect.bottom<=viewport.bottom
        attempts.append({'native_bounds':list(native),'visible':control['visible'],
                         'tick':observed['runtime']['ui_update']['ticks']})
        assert bool(control['visible'])==visible,(label,'Native and diagnostic visibility disagree')
        if visible:break
        assert native!=last_rect,(label,'Touch pan did not move the clipped action')
        last_rect=native
        viewport_rect=(viewport.left,viewport.top,viewport.right,viewport.bottom)
        hit_tests=attempts[-1]['pan_hit_tests']=[]
        def is_body(x,y):
            hit=ui.WindowFromPoint(ui.W.POINT(x,y))
            name=C.create_unicode_buffer(256)
            if hit:assert ui.GetClassNameW(hit,name,len(name))
            hit_tests.append({'point':[x,y],'hwnd':int(hit or 0),
                              'class':name.value,'is_body':hit==body})
            return hit==body
        path=preferences_pan_path(viewport_rect,native,scale,is_body)
        native_class=C.create_unicode_buffer(256)
        assert ui.GetClassNameW(body,native_class,len(native_class))
        attempts[-1]['pan']={'start':list(path[0]),'end':list(path[1]),
            'body_bounds':list(viewport_rect),'native_hit_hwnd':int(body),
            'native_hit_class':native_class.value,'hit_is_body':True}
        # The body owns this gesture. An Edit child may consume it as text
        # interaction, even though IsChild(body, hit) would have passed.
        assert foreground()==popup,'Preferences lost foreground before pan'
        assert ui.WindowFromPoint(ui.W.POINT(*path[0]))==body
        assert ui.WindowFromPoint(ui.W.POINT(*path[1]))==body
        assert dpi('--pan',*path[0],*path[1])['touch_injected']
        time.sleep(.4)
    else:raise AssertionError(label+': Preferences action cannot be reached by bounded touch scroll')
    assert foreground()==popup and ui.IsWindowEnabled(target)
    current=bounds(target)
    assert (current.left,current.top,current.right,current.bottom)==native,'Preferences moved before the tap'
    return observed,target,{'label':label,'observations':attempts,'fully_visible':True}

def touch_preferences_action(label,scale,*,ui,pid,observe,bounds,dpi,report):
    """Reach a real drawer action before tapping; never message a clipped HWND."""
    observed,target,result=reach_preferences_action(label,scale,ui=ui,pid=pid,observe=observe,bounds=bounds,dpi=dpi,report=report)
    control=next(c for c in observed['runtime']['display']['interaction_controls']
                 if c['label']==label)
    rect=bounds(target)
    assert (rect.left,rect.top,rect.right,rect.bottom)==(control['x'],control['y'],
        control['x']+control['width'],control['y']+control['height']), 'Preferences moved before the tap'
    point=ui.W.POINT((rect.left+rect.right)//2,(rect.top+rect.bottom)//2)
    assert ui.WindowFromPoint(point)==target,'Another surface covers the Preferences action'
    assert dpi('--tap',point.x,point.y)['touch_injected']
    result.update(native_hit_target_verified=True,tap=[point.x,point.y],touch_injected=True)
    return result

