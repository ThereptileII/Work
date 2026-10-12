"""Response boundaries for the informational site; no vendor scripts or intake."""


class MarketingSecurityHeaders:
    def __init__(self, get_response):
        self.get_response = get_response

    def __call__(self, request):
        response = self.get_response(request)
        match = getattr(request, "resolver_match", None)
        if match is not None and match.app_name == "marketing":
            response["Content-Security-Policy"] = (
                "default-src 'self'; script-src 'self'; style-src 'self'; "
                "img-src 'self' data:; font-src 'self'; connect-src 'self'; "
                "object-src 'none'; base-uri 'self'; form-action 'self'; "
                "frame-ancestors 'none'"
            )
            response["Referrer-Policy"] = "strict-origin-when-cross-origin"
            response["Permissions-Policy"] = "camera=(), microphone=(), geolocation=()"
            response["X-Content-Type-Options"] = "nosniff"
            response["X-Robots-Tag"] = "noindex, nofollow"
        return response
