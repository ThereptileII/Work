"""Read-only admission of one independently approved interactive GitHub runner.

Runner environment variables are comparisons, never host authorization. The
original schema-2 declaration is bound by the existing postproduction approval.
This module creates no profile, process, approval or installed state.
"""
import re

import installed_customer_launch as customer

REPOSITORY = 'ThereptileII/Work'
MODE = 'interactive-self-hosted'
DECLARATION_KEYS = frozenset(('schema', 'b0', 'setup', 'operatorKit', 'clientRequest',
    'brokerBootstrapPolicy', 'approvedBaseline', 'trustConfig', 'bootstrapCache',
    'artifactDirectory', 'preserveSettings'))
FAILURE = 'Installed interactive host admission refused'


def require(value):
    if not value:
        raise ValueError(FAILURE)


def target(value):
    require(type(value) is dict and set(value) ==
            {'kind', 'runnerId', 'runnerGroupId', 'labels', 'clientSid'} and value['kind'] == MODE)
    for key in ('runnerId', 'runnerGroupId'):
        require(type(value[key]) is str and
                re.fullmatch(r'[1-9][0-9]{0,19}' if key == 'runnerId' else r'(?:0|[1-9][0-9]{0,19})', value[key]))
    labels = value['labels']
    require(type(labels) is list and 4 <= len(labels) <= 8 and
            all(type(label) is str and re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,63}', label) for label in labels) and
            len(set(labels)) == len(labels) and
            {'self-hosted', 'Windows', 'X64', 'skager-disposable-interactive'} <= set(labels))
    require(type(value['clientSid']) is str and
            re.fullmatch(r'S-1-5-21-[1-9][0-9]*-[1-9][0-9]*-[1-9][0-9]*-[1-9][0-9]*', value['clientSid']))
    return value


def declaration(value):
    require(type(value) is dict and type(value.get('schema')) is int)
    if value['schema'] == 1:
        require(set(value) == DECLARATION_KEYS)
        return None  # Existing declarations grant no self-hosted admission.
    require(value['schema'] == 2 and set(value) == DECLARATION_KEYS | {'hostAdmission'})
    return target(value['hostAdmission'])


def environment(values, platform, mode='github-hosted'):
    require(platform == 'win32' and mode in ('github-hosted', MODE) and
            values.get('GITHUB_ACTIONS') == 'true' and values.get('RUNNER_OS') == 'Windows' and
            values.get('GITHUB_REPOSITORY') == REPOSITORY and
            values.get('GITHUB_EVENT_NAME') == 'workflow_dispatch' and
            values.get('RUNNER_ENVIRONMENT') == ('self-hosted' if mode == MODE else 'github-hosted') and
            (mode != MODE or values.get('RUNNER_ARCH') == 'X64'))


def validate_live(approved, runner, workflow, values, platform, read_api):
    """Authenticate the exact live pending job and observe original native UAC capability."""
    approved = target(approved)
    environment(values, platform, MODE)
    require(type(runner) is dict and set(runner) ==
            {'repository', 'headSha', 'runId', 'runAttempt', 'jobId', 'jobName'} and
            runner['repository'] == REPOSITORY and
            type(workflow) is str and re.fullmatch(r'\.github/workflows/[A-Za-z0-9_.-]+\.yml', workflow))
    require(all(values.get(env) == runner[key] for env, key in
                (('GITHUB_REPOSITORY', 'repository'), ('GITHUB_SHA', 'headSha'),
                 ('GITHUB_RUN_ID', 'runId'), ('GITHUB_RUN_ATTEMPT', 'runAttempt'))))
    require(type(runner['headSha']) is str and re.fullmatch(r'[0-9a-f]{40}', runner['headSha']))
    for key in ('runId', 'runAttempt', 'jobId'):
        require(type(runner[key]) is str and re.fullmatch(r'[1-9][0-9]{0,19}', runner[key]))
    require(type(runner['jobName']) is str and 0 < len(runner['jobName']) <= 200)
    endpoint = f'repos/{REPOSITORY}/actions/runs/{runner["runId"]}/attempts/{runner["runAttempt"]}'
    read = lambda path: read_api(path, timeout_seconds=30, max_bytes=256 << 10)
    run = read(endpoint)
    require(type(run) is dict and str(run.get('id')) == runner['runId'] and
            str(run.get('run_attempt')) == runner['runAttempt'] and run.get('head_sha') == runner['headSha'] and
            run.get('path') == workflow and run.get('event') == 'workflow_dispatch' and
            run.get('status') == 'in_progress' and run.get('conclusion') is None and
            type(run.get('head_repository')) is dict and run['head_repository'].get('full_name') == REPOSITORY)
    jobs = read(endpoint + '/jobs?per_page=100&page=1')
    require(type(jobs) is dict and type(jobs.get('total_count')) is int and type(jobs.get('jobs')) is list and
            jobs['total_count'] == len(jobs['jobs']) <= 100 and all(type(job) is dict for job in jobs['jobs']))
    matches = [job for job in jobs['jobs'] if str(job.get('id')) == runner['jobId']]
    require(len(matches) == 1)
    job = matches[0]
    require(job.get('name') == runner['jobName'] and job.get('status') == 'in_progress' and
            job.get('conclusion') is None and str(job.get('run_id')) == runner['runId'] and
            str(job.get('run_attempt')) == runner['runAttempt'] and job.get('head_sha') == runner['headSha'] and
            type(job.get('runner_id')) is int and str(job['runner_id']) == approved['runnerId'] and
            type(job.get('runner_group_id')) is int and str(job['runner_group_id']) == approved['runnerGroupId'] and
            type(job.get('labels')) is list and all(type(label) is str for label in job['labels']) and
            sorted(job['labels']) == sorted(approved['labels']))
    observed = customer.inspect_customer_capability()
    require(observed.get('state') == 'available' and observed.get('normalTokenAvailable') is True and
            observed.get('consentPromptAvailable') is True and
            observed.get('nativeCustomerAcceptance') is False and observed.get('consentPromptAcceptance') is False and
            observed.get('observations', {}).get('current', {}).get('sid') == approved['clientSid'])
