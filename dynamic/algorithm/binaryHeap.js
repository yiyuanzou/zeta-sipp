// OPEN nodes have a mutable f key and need direct decrease-key lookup.
// A node may belong to only one live OpenHeap and may occur there only once.
class OpenHeap {
    constructor() {
        this.content = [];
    }

    size() {
        return this.content.length;
    }

    push(node) {
        this.content.push(node);
        this.bubbleUp(this.content.length - 1);
    }

    sortElement(node) {
        // Resident keys may only decrease. Revalidation may increase a key
        // after removal, in which case the node is inserted again with push.
        const index = node.heapIndex;
        if (index >= 0 && this.content[index] === node) this.bubbleUp(index);
    }

    shift() {
        const a = this.content, first = a[0];
        if (first === undefined) return undefined;

        const end = a.pop();
        first.heapIndex = -1;
        if (a.length) {
            a[0] = end;
            this.sinkDown(0);
        }
        return first;
    }

    bubbleUp(index) {
        const a = this.content, node = a[index], score = node.f;

        // Move parents down into the hole, then place the node once.
        while (index > 0) {
            const parentIndex = (index - 1) >> 1, parent = a[parentIndex];
            if (score >= parent.f) break;

            a[index] = parent;
            parent.heapIndex = index;
            index = parentIndex;
        }

        a[index] = node;
        node.heapIndex = index;
    }

    sinkDown(index) {
        const a = this.content, node = a[index], score = node.f, length = a.length;

        while (index * 2 + 1 < length) {
            let child = index * 2 + 1;
            let childScore = a[child].f;

            // Strict comparison preserves the original left-first tie order.
            if (child + 1 < length && a[child + 1].f < childScore) {
                child++;
                childScore = a[child].f;
            }
            if (childScore >= score) break;

            a[index] = a[child];
            a[index].heapIndex = index;
            index = child;
        }

        a[index] = node;
        node.heapIndex = index;
    }
}

// SCAN and BOUND only insert and remove entries; resident keys stay unchanged.
// Share their ordering logic without writing an index into every entry.
class MinHeap {
    constructor(scoreKey) {
        this.content = [];
        this.scoreKey = scoreKey;
    }

    size() {
        return this.content.length;
    }

    push(element) {
        this.content.push(element);
        this.bubbleUp(this.content.length - 1);
    }

    shift() {
        const a = this.content, first = a[0];
        if (first === undefined) return undefined;

        const end = a.pop();
        if (a.length) {
            a[0] = end;
            this.sinkDown(0);
        }
        return first;
    }

    bubbleUp(index) {
        const a = this.content, key = this.scoreKey, element = a[index], score = element[key];

        while (index > 0) {
            const parentIndex = (index - 1) >> 1, parent = a[parentIndex];
            if (score >= parent[key]) break;

            a[index] = parent;
            index = parentIndex;
        }

        a[index] = element;
    }

    sinkDown(index) {
        const a = this.content, key = this.scoreKey, element = a[index];
        const score = element[key], length = a.length;

        // Move the smaller child into the hole; equal children keep left first.
        while (index * 2 + 1 < length) {
            let child = index * 2 + 1;
            let childScore = a[child][key];
            if (child + 1 < length && a[child + 1][key] < childScore) {
                child++;
                childScore = a[child][key];
            }
            if (childScore >= score) break;

            a[index] = a[child];
            index = child;
        }

        a[index] = element;
    }
}

module.exports = { OpenHeap, MinHeap };
