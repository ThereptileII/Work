use strict;
use warnings;
use JSON::PP;
use runner;
use servers;

# Exercise the unmodified locked upstream modules, without a network listener.
# Run each operation in a separate bounded process from the Python controller.
my ($operation, $directory) = @ARGV;
chdir $directory or die "Cannot enter disposable directory: $!";
mkdir 'logs' or die "Cannot create log directory: $!";
$globalconfig::LOGDIR = 'logs';
if ($operation eq 'readiness') {
    my $id = runner::runner_init('logs', 0);
    my $sent = runner::runnerac_stopservers($id); # No servers were started.
    my @ready = runner::runnerar_ready(0.1);
    # The integrated call above already wrote its reply. Directly reading it
    # also proves a missing readiness notification is not a missing response.
    my @response = runner::runnerar($id);
    print encode_json({os => $^O, runner => $id, sent => $sent,
                       ready => \@ready, response => \@response}), "\n";
    runner::runnerac_shutdown($id);
} elsif ($operation eq 'server') {
    open(my $marker, '>', 'marker.pl') or die "Cannot write marker: $!";
    print $marker 'open(my $p, ">", "marker.pid") or die $!; print $p "$$\n"; close $p; sleep 3;';
    close $marker or die "Cannot close marker: $!";
    my ($child, $pid) = servers::startnew('"' . $^X . '" marker.pl', 'marker.pid', 2, 0);
    waitpid($child, 0) if $child && $child != -1;
    print encode_json({os => $^O, child => $child, pid => $pid,
                       marker => -s 'marker.pid' ? JSON::PP::true : JSON::PP::false}), "\n";
} else {
    die "Unknown operation";
}
