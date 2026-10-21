-- Schedule the store job (email outbox, retries, stale orders, expired sessions)
-- once a minute with pg_cron + pg_net. Apply in the Supabase SQL editor AFTER
-- store.sql. The job secret is read from Supabase Vault at run time and is
-- never stored in Git or in the cron command text. Create it first, typing the
-- value yourself (the same value as the SKAGER_JOB_SECRET function secret):
--   select vault.create_secret('<at least 32 random characters>', 'skager_job_secret');
create extension if not exists pg_cron;
create extension if not exists pg_net;

select cron.unschedule(jobid) from cron.job where jobname = 'skager-store-tick';
select cron.schedule('skager-store-tick', '* * * * *', $job$
  select net.http_post(
    url := 'https://kelwgubkuvditalsbqpr.supabase.co/functions/v1/skager-web/api/jobs/tick',
    headers := jsonb_build_object('x-skager-job',
      (select decrypted_secret from vault.decrypted_secrets where name = 'skager_job_secret')),
    timeout_milliseconds := 20000);
$job$);
