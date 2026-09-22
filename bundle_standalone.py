import re
import os

def build_standalone():
    base_dir = os.path.dirname(os.path.abspath(__file__))

    with open(os.path.join(base_dir, 'index.html'), 'r', encoding='utf-8') as f:
        html = f.read()

    with open(os.path.join(base_dir, 'style.css'), 'r', encoding='utf-8') as f:
        css = f.read()

    three_standalone_path = os.path.join(base_dir, 'three.standalone.js')
    if not os.path.exists(three_standalone_path):
        import urllib.request
        print("Downloading three.standalone.js (Three.js 0.160.0 UMD bundle)...")
        req = urllib.request.Request('https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.min.js', headers={'User-Agent': 'Mozilla/5.0'})
        with urllib.request.urlopen(req) as resp:
            with open(three_standalone_path, 'wb') as f_out:
                f_out.write(resp.read())

    with open(three_standalone_path, 'r', encoding='utf-8') as f:
        three_js = f.read()

    with open(os.path.join(base_dir, 'dashboard.js'), 'r', encoding='utf-8') as f:
        dash_js = f.read()

    # 1. three.standalone.js is a complete UMD bundle natively providing window.THREE
    three_converted = three_js

    # 2. Convert dashboard.js to consume window.THREE directly
    dash_converted = re.sub(r'import\s+\*\s+as\s+THREE\s+from\s+["\']three["\'];', 'const THREE = window.THREE;', dash_js)

    # 3. Replace <link rel="stylesheet" href="style.css"> with inline <style>
    html = re.sub(r'<link\s+rel=["\']stylesheet["\']\s+href=["\']style\.css["\']\s*>', '<style>\n' + css + '\n</style>', html)

    # 4. Remove importmap
    html = re.sub(r'<script\s+type=["\']importmap["\']>[\s\S]*?</script>', '', html)

    # 5. Replace <script type="module" src="dashboard.js"></script> with inline scripts
    inline_scripts = '<script>\n' + three_converted + '\n</script>\n<script>\n' + dash_converted + '\n</script>'
    html = re.sub(r'<script\s+type=["\']module["\']\s+src=["\']dashboard\.js["\']\s*>\s*</script>', lambda m: inline_scripts, html)

    out_path = os.path.join(base_dir, 'PGK_Simulation_Standalone.html')
    with open(out_path, 'w', encoding='utf-8') as f:
        f.write(html)

    size_mb = os.path.getsize(out_path) / (1024 * 1024)
    print(f"Successfully generated {out_path} ({size_mb:.2f} MB)")

if __name__ == '__main__':
    build_standalone()
