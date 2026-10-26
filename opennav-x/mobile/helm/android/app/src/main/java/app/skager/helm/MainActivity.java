package app.skager.helm;

import android.app.Activity;
import android.content.SharedPreferences;
import android.graphics.Color;
import android.net.Uri;
import android.os.Bundle;
import android.view.Gravity;
import android.view.View;
import android.view.Window;
import android.webkit.SslErrorHandler;
import android.webkit.WebChromeClient;
import android.webkit.WebResourceError;
import android.webkit.WebResourceRequest;
import android.webkit.WebSettings;
import android.webkit.WebView;
import android.webkit.WebViewClient;
import android.widget.Button;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.TextView;

/**
 * Small native Android shell for the Helm LAN client.
 *
 * The PC remains the navigation/rendering authority. The WebView hosts the
 * same WebRTC probe used by the server, which keeps this APK free of a second
 * chart or navigation implementation while native Android/iOS media shells
 * mature. No certificate errors are accepted and no actuator permission is
 * granted by this activity.
 */
public final class MainActivity extends Activity {
    private static final String PREFS = "skager_helm";
    private static final String HOST = "host";

    private EditText hostField;
    private TextView status;
    private WebView webView;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        requestWindowFeature(Window.FEATURE_NO_TITLE);
        getWindow().setStatusBarColor(Color.rgb(13, 16, 18));
        getWindow().setNavigationBarColor(Color.rgb(13, 16, 18));
        setContentView(createContent());
    }

    private View createContent() {
        LinearLayout root = new LinearLayout(this);
        root.setOrientation(LinearLayout.VERTICAL);
        root.setBackgroundColor(Color.rgb(13, 16, 18));

        LinearLayout connectBar = new LinearLayout(this);
        connectBar.setGravity(Gravity.CENTER_VERTICAL);
        connectBar.setPadding(dp(12), dp(8), dp(12), dp(8));

        hostField = new EditText(this);
        hostField.setSingleLine(true);
        hostField.setTextColor(Color.rgb(245, 247, 248));
        hostField.setHintTextColor(Color.rgb(156, 174, 184));
        hostField.setHint("SKAGER PC address (http:// or https://)");
        hostField.setText(getPreferences(0).getString(HOST, "http://192.168.0.2:43872"));
        hostField.setSelectAllOnFocus(false);
        connectBar.addView(hostField, new LinearLayout.LayoutParams(0, dp(52), 1));

        Button connect = new Button(this);
        connect.setText("Open Helm");
        connect.setTextColor(Color.rgb(6, 16, 20));
        connect.setOnClickListener(v -> openHelm());
        connectBar.addView(connect, new LinearLayout.LayoutParams(dp(124), dp(52)));
        root.addView(connectBar);

        status = new TextView(this);
        status.setText("Enter the address of the SKAGER PC on the local network.");
        status.setTextColor(Color.rgb(156, 174, 184));
        status.setPadding(dp(16), 0, dp(16), dp(8));
        root.addView(status, new LinearLayout.LayoutParams(-1, -2));

        webView = new WebView(this);
        configureWebView();
        root.addView(webView, new LinearLayout.LayoutParams(-1, 0, 1));
        return root;
    }

    private void configureWebView() {
        WebSettings settings = webView.getSettings();
        settings.setJavaScriptEnabled(true);
        settings.setDomStorageEnabled(true);
        settings.setMediaPlaybackRequiresUserGesture(false);
        settings.setAllowFileAccess(false);
        settings.setAllowContentAccess(false);
        settings.setMixedContentMode(WebSettings.MIXED_CONTENT_NEVER_ALLOW);
        webView.setBackgroundColor(Color.BLACK);
        webView.setWebViewClient(new WebViewClient() {
            @Override
            public void onPageStarted(WebView view, String url, android.graphics.Bitmap favicon) {
                status.setText("Opening Helm…");
            }

            @Override
            public void onPageFinished(WebView view, String url) {
                status.setText("Helm ready — enter the pairing code in the client.");
            }

            @Override
            public void onReceivedError(WebView view, WebResourceRequest request, WebResourceError error) {
                if (request.isForMainFrame()) status.setText("Could not open Helm on this address.");
            }

            @Override
            public void onReceivedSslError(WebView view, SslErrorHandler handler, android.net.http.SslError error) {
                handler.cancel();
                status.setText("TLS certificate rejected; use the Helm server's valid certificate.");
            }
        });
        webView.setWebChromeClient(new WebChromeClient());
    }

    private void openHelm() {
        String raw = hostField.getText().toString().trim();
        Uri uri = Uri.parse(raw);
        String scheme = uri.getScheme();
        if (uri.getHost() == null || !("http".equalsIgnoreCase(scheme) || "https".equalsIgnoreCase(scheme))) {
            status.setText("Use a full LAN address beginning with http:// or https://.");
            return;
        }
        getPreferences(0).edit().putString(HOST, raw).apply();
        String base = raw.endsWith("/") ? raw.substring(0, raw.length() - 1) : raw;
        status.setText("Connecting to " + uri.getHost() + "…");
        webView.loadUrl(base + "/helm/");
    }

    @Override
    public void onBackPressed() {
        if (webView != null && webView.canGoBack()) {
            webView.goBack();
        } else {
            super.onBackPressed();
        }
    }

    private int dp(int value) {
        return Math.round(value * getResources().getDisplayMetrics().density);
    }
}
