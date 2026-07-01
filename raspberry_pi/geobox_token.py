#!/usr/bin/env python3
"""
geobox_token.py -- minimal connectivity test: log in to GEOMind (geobox SDK) and
print the access token. Use this to confirm credentials before running
monitoring_control.py.

    pip install geobox tqdm requests   # tqdm is an undeclared geobox dependency

    python3 geobox_token.py                                  # uses the defaults below
    python3 geobox_token.py --user USER --pass PASS
    python3 geobox_token.py --apikey YOUR_API_KEY
    python3 geobox_token.py --host https://app.geo-mind.ai

NOTE: a wrong password counts as a failed login. Do NOT run this in a loop --
GEOMind can temporarily block the IP / disable the account after repeated
failures. If it fails, fix the credentials and try once more.
"""

import argparse
import sys

try:
    from geobox import GeoboxClient
except Exception as e:                       # most likely: missing 'tqdm'
    print("cannot import geobox:", e)
    print("install deps:  pip install geobox tqdm requests")
    sys.exit(1)


def main():
    ap = argparse.ArgumentParser(description="Get a GEOMind/geobox access token")
    ap.add_argument("--host", default="https://app.geo-mind.ai")
    ap.add_argument("--user", default="PDO@excavator1")
    ap.add_argument("--pass", dest="password", default="PDO@excavator1")
    ap.add_argument("--apikey", default=None,
                    help="authenticate with an API key instead of user/password")
    ap.add_argument("--insecure", action="store_true",
                    help="skip TLS certificate verification")
    args = ap.parse_args()

    print(f"connecting to {args.host} ...")
    try:
        if args.apikey:
            client = GeoboxClient(host=args.host, apikey=args.apikey,
                                  verify=not args.insecure)
        else:
            client = GeoboxClient(host=args.host, username=args.user,
                                  password=args.password, verify=not args.insecure)
    except Exception as e:
        print("LOGIN FAILED:", e)
        print("-> check the username (might be an email), the password, or pass "
              "--apikey. Do not retry repeatedly (lockout risk).")
        sys.exit(1)

    print("LOGIN OK")
    token = getattr(client, "access_token", None)
    if token:
        print("access_token:", token)
    else:
        print("(apikey mode -- no bearer token; requests carry the apikey)")


if __name__ == "__main__":
    main()
