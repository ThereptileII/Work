-- Disposable CI database ONLY: rollback-only assertions for store-steps.sql.
BEGIN;
DO $test$
DECLARE catalog jsonb; access_id uuid; gps_id uuid; pc_id uuid; step jsonb; product jsonb; created jsonb;
BEGIN
  catalog := public.store_catalog();
  SELECT (p->>'id')::uuid INTO access_id FROM jsonb_array_elements(catalog->'products') p WHERE p->>'slug' = 'skager-beta-access';
  SELECT (p->>'id')::uuid INTO gps_id FROM jsonb_array_elements(catalog->'products') p WHERE p->>'slug' = 'gps-module';
  SELECT (p->>'id')::uuid INTO pc_id FROM jsonb_array_elements(catalog->'products') p WHERE p->>'slug' = 'helm-pc';
  IF (SELECT count(*) FROM jsonb_array_elements(catalog->'steps') s WHERE s->>'flow' = 'checkout') <> 2
    OR (SELECT count(*) FROM jsonb_array_elements(catalog->'steps') s WHERE s->>'flow' = 'download') <> 1
    OR (SELECT count(*) FROM jsonb_array_elements(catalog->'products') p WHERE p->>'step_id' IS NOT NULL AND (p->>'checkout_addon')::boolean) <> 2 THEN
    RAISE EXCEPTION 'Seed steps incomplete: %', catalog->'steps';
  END IF;

  -- The seeded helm PC is now offered in the checkout.
  created := public.store_create_order('access','steps@example.com',NULL,jsonb_build_array(jsonb_build_object('product_id',access_id,'quantity',1),
    jsonb_build_object('product_id',pc_id,'quantity',1), jsonb_build_object('product_id',gps_id,'quantity',1)),NULL,NULL);
  IF created->>'state' <> 'created' OR (created->>'subtotal_minor')::int <> 2000 + 54900 + 4900 THEN RAISE EXCEPTION 'Step order wrong: %', created; END IF;

  -- An admin-added stage brings its product into the checkout; hiding the stage takes it out again.
  step := public.store_admin_save_step('admin@example.com',jsonb_build_object('flow','checkout','title','Cables','skip_label','No cables needed','sort',30));
  product := public.store_admin_save_product('admin@example.com',jsonb_build_object('slug','nmea-cable-x1','kind','hardware','name','NMEA cable',
    'price_minor',1900,'shippable',true,'step_id',step->>'id'));
  IF NOT (product->>'checkout_addon')::boolean OR product->>'step_id' <> step->>'id' THEN RAISE EXCEPTION 'Product not in step: %', product; END IF;
  step := public.store_admin_save_step('admin@example.com',step || jsonb_build_object('active',false));
  IF (SELECT checkout_addon FROM skager_private.store_products WHERE id = (product->>'id')::uuid)
    OR EXISTS (SELECT 1 FROM jsonb_array_elements(public.store_catalog()->'steps') s WHERE s->>'id' = step->>'id') THEN
    RAISE EXCEPTION 'Hidden step still offered';
  END IF;
  BEGIN
    PERFORM public.store_create_order('access','hidden@example.com',NULL,jsonb_build_array(jsonb_build_object('product_id',access_id,'quantity',1),
      jsonb_build_object('product_id',(product->>'id')::uuid,'quantity',1)),NULL,NULL);
    RAISE EXCEPTION 'Product of a hidden step accepted';
  EXCEPTION WHEN raise_exception THEN IF SQLERRM LIKE 'Product of%' THEN RAISE; END IF; END;

  -- Products only join checkout stages; download stages need an https link with a label.
  step := public.store_admin_save_step('admin@example.com',jsonb_build_object('flow','download','title','Charts','skip_label','I have charts',
    'link_url','https://o-charts.org/','link_label','Get charts'));
  BEGIN
    PERFORM public.store_admin_save_product('admin@example.com',jsonb_build_object('id',gps_id,'name','GPS','price_minor',4900,'step_id',step->>'id'));
    RAISE EXCEPTION 'Product put in a download step';
  EXCEPTION WHEN raise_exception THEN IF SQLERRM LIKE 'Product put%' THEN RAISE; END IF; END;
  BEGIN
    PERFORM public.store_admin_save_step('admin@example.com',jsonb_build_object('flow','download','title','X','skip_label','Skip',
      'link_url','javascript:alert(1)','link_label','Go'));
    RAISE EXCEPTION 'Unsafe link accepted';
  EXCEPTION WHEN check_violation THEN NULL; END;
  BEGIN
    PERFORM public.store_admin_save_step('admin@example.com',jsonb_build_object('flow','download','title','X','skip_label','Skip',
      'link_url','https://example.com/'));
    RAISE EXCEPTION 'Link without label accepted';
  EXCEPTION WHEN check_violation THEN NULL; END;
  BEGIN
    PERFORM public.store_admin_save_step('admin@example.com',jsonb_build_object('flow','shop','title','X','skip_label','Skip'));
    RAISE EXCEPTION 'Unknown flow accepted';
  EXCEPTION WHEN check_violation THEN NULL; END;

  -- A product moved out of every stage leaves the checkout.
  product := public.store_admin_save_product('admin@example.com',jsonb_build_object('id',gps_id,'name','SKAGER GPS module','price_minor',4900,'shippable',true));
  IF (product->>'checkout_addon')::boolean OR product->>'step_id' IS NOT NULL THEN RAISE EXCEPTION 'Product kept in checkout: %', product; END IF;

  IF has_function_privilege('anon','public.store_admin_save_step(text,jsonb)','EXECUTE')
    OR has_function_privilege('authenticated','public.store_admin_save_step(text,jsonb)','EXECUTE')
    OR NOT has_function_privilege('service_role','public.store_admin_save_step(text,jsonb)','EXECUTE')
    OR has_table_privilege('service_role','skager_private.store_steps','SELECT')
    OR has_function_privilege('service_role','skager_private.store_step_json(skager_private.store_steps)','EXECUTE') THEN
    RAISE EXCEPTION 'Step privilege boundary wrong';
  END IF;
END;
$test$;
ROLLBACK;
