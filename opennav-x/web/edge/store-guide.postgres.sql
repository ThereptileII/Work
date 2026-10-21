-- Disposable CI database ONLY: rollback-only assertions for store-guide.sql.
BEGIN;
DO $test$
DECLARE catalog jsonb; step jsonb;
BEGIN
  catalog := public.store_catalog();
  IF (SELECT count(*) FROM jsonb_array_elements(catalog->'steps') s WHERE s->>'flow' = 'guide') <> 2
    OR NOT EXISTS (SELECT 1 FROM jsonb_array_elements(catalog->'steps') s WHERE s->>'flow' = 'guide' AND s->>'summary' LIKE E'%\n%') THEN
    RAISE EXCEPTION 'Seed guide incomplete: %', catalog->'steps';
  END IF;

  -- Admins add guide steps with line breaks; other control characters stay out.
  step := public.store_admin_save_step('admin@example.com',jsonb_build_object('flow','guide','title','Charts','skip_label','Next',
    'summary',E'Install charts.\nOpen the chart manager.'));
  IF step->>'flow' <> 'guide' OR step->>'summary' <> E'Install charts.\nOpen the chart manager.' THEN RAISE EXCEPTION 'Guide step wrong: %', step; END IF;
  BEGIN
    PERFORM public.store_admin_save_step('admin@example.com',jsonb_build_object('flow','guide','title','X','skip_label','Next','summary',E'a\tb'));
    RAISE EXCEPTION 'Tab accepted';
  EXCEPTION WHEN check_violation THEN NULL; END;
  BEGIN
    PERFORM public.store_admin_save_step('admin@example.com',jsonb_build_object('flow','guide','title',E'X\nY','skip_label','Next'));
    RAISE EXCEPTION 'Line break in title accepted';
  EXCEPTION WHEN check_violation THEN NULL; END;
  BEGIN
    PERFORM public.store_admin_save_step('admin@example.com',jsonb_build_object('flow','guide','title','X','skip_label','Next','summary',repeat('a',1201)));
    RAISE EXCEPTION 'Overlong guide accepted';
  EXCEPTION WHEN check_violation THEN NULL; END;

  -- Products only ever join checkout steps.
  BEGIN
    PERFORM public.store_admin_save_product('admin@example.com',jsonb_build_object('slug','guide-x1','kind','hardware','name','X',
      'price_minor',100,'step_id',step->>'id'));
    RAISE EXCEPTION 'Product put in a guide step';
  EXCEPTION WHEN raise_exception THEN IF SQLERRM LIKE 'Product put%' THEN RAISE; END IF; END;
END;
$test$;
ROLLBACK;
