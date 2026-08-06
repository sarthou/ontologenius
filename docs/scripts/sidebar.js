document.addEventListener('DOMContentLoaded', () => {
    // 1. Select ALL elements with a data-sidebar attribute
    const containers = document.querySelectorAll('[data-sidebar]');
    if (containers.length === 0) return;

    const normalize = (urlStr) => {
        try {
            const url = new URL(urlStr, window.location.href);
            return url.pathname
                .toLowerCase()
                .replace(/\/$/, '')
                .replace(/\.html?$/i, '');
        } catch (e) {
            return '';
        }
    };

    const currentNormalized = normalize(window.location.href);

    // 2. Loop through each container and fetch its respective file
    containers.forEach(container => {
        const sidebarFile = container.getAttribute('data-sidebar');
        if (!sidebarFile) return;

        fetch(sidebarFile)
            .then(response => {
                if (!response.ok) throw new Error(`Could not load: ${sidebarFile}`);
                return response.text();
            })
            .then(html => {
                container.innerHTML = html;

                const links = container.querySelectorAll('a');
                links.forEach(link => {
                    const href = link.getAttribute('href');
                    if (!href) return;

                    if (normalize(href) === currentNormalized) {
                        link.style.fontWeight = 'bold';
                        link.classList.add('active');

                        if (link.parentElement.tagName === 'LI') {
                            link.parentElement.classList.add('active-item');
                        }
                    }
                });
            })
            .catch(error => console.error('Sidebar load error:', error));
    });
});