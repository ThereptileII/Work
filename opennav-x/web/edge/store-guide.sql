-- SKAGER "Get going" guide: admin-defined steps shown after the SKAGER download in the
-- customer account, such as setting up a GRIBstream token for forecast wind or an
-- AISstream key for online AIS. Apply after store-steps.sql, as the database owner, in
-- one transaction. Additive: existing steps, products and orders are unchanged.
begin;

alter table skager_private.store_steps drop constraint store_steps_flow_check;
alter table skager_private.store_steps add constraint store_steps_flow_check check (flow in ('checkout','download','guide'));
-- Guide text needs line breaks: the first line is the introduction, each further line a numbered step.
alter table skager_private.store_steps drop constraint store_steps_summary_check;
alter table skager_private.store_steps add constraint store_steps_summary_check
  check (char_length(summary) <= 1200 and translate(summary, E'\n', '') !~ '[[:cntrl:]]');

-- Seed: the two online services. Both are optional and use the customer's own key.
insert into skager_private.store_steps(flow,title,summary,skip_label,link_url,link_label,sort) values
('guide','Forecast wind with GRIBstream',
 'Optional: see forecast wind at your boat, on the chart and along a route, using your own free GRIBstream token. Forecast wind is in SKAGER 0.4.0-beta2.4 and later.' || E'\n' ||
 'Create a free account at gribstream.com and copy your API token from the dashboard.' || E'\n' ||
 'In SKAGER, open Settings → Navigation → Weather, turn it on, paste the token and choose Save.' || E'\n' ||
 'Choose Test connection. The token is kept in Windows Credential Manager and never shown again.' || E'\n' ||
 'The free plan has a daily credit limit. If forecasts stop for the rest of the day, the limit is reached; a Pro plan raises it.' || E'\n' ||
 'Forecast wind is advisory model output and always shown apart from the wind you measure on board.',
 'Skip for now','https://gribstream.com/app/dashboard','Get a GRIBstream token',10),
('guide','Online AIS with AISstream',
 'Optional: see ships beyond the range of your own AIS receiver, using your own AISstream key.' || E'\n' ||
 'Sign in at aisstream.io, open Account → API keys and create a key. It is shown only once, so copy it straight away.' || E'\n' ||
 'In SKAGER, open Traffic → Online AIS settings → Set AISStream key, paste the key and choose Save key.' || E'\n' ||
 'Choose Enabled when you want internet traffic for the chart area. Saving the key does not turn it on.' || E'\n' ||
 'Keep the key to yourself: leave it out of emails, feedback and screenshots.' || E'\n' ||
 'Internet traffic can be delayed or incomplete. It adds to your onboard AIS and never replaces it or a proper lookout.',
 'Skip for now','https://aisstream.io/authenticate','Get an AISstream key',20);
commit;
