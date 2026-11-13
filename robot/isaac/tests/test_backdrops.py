"""The backdrop list's rules, offline: which rows qualify, what a download must be, and the seeded pick."""

from teleop_isaac import backdrops

HEADER = "ImageID,Subset,OriginalURL,OriginalLandingURL,License,AuthorProfileURL,Author,Title,OriginalSize,OriginalMD5,Thumbnail300KURL,Rotation\n"


def row(image_id, licence=backdrops.CC_BY_2, rotation="0.0"):
    return f"{image_id},validation,u,https://flickr/x,{licence},https://flickr/p,An Author,A title,1,md5,t,{rotation}\n"


def test_only_cc_by_2_unrotated_rows_with_real_ids_qualify_in_id_order(tmp_path):
    csv = tmp_path / "meta.csv"
    csv.write_text(HEADER + row("ffffffffffffffff") + row("0000000000000001") +
                   row("0000000000000002", licence="https://creativecommons.org/licenses/by-nc/2.0/") +
                   row("0000000000000003", rotation="90.0") + row("../../etc/passwd") + row("0000000000000004", rotation=""))
    ids = [r["ImageID"] for r in backdrops.candidates(str(csv))]
    assert ids == ["0000000000000001", "0000000000000004", "ffffffffffffffff"]


def test_a_download_must_look_like_a_jpeg_and_not_be_huge():
    assert backdrops.check_image(b"<html>nope</html>") is None
    assert backdrops.check_image(b"\xff\xd8\xff" + b"\x00" * (backdrops.MAX_BYTES + 1)) is None


def test_the_pick_is_two_different_images_from_the_seed_and_the_change_alone():
    images = [{"sha256": str(i)} for i in range(50)]
    a = backdrops.pick(images, 7, 3)
    assert a == backdrops.pick(images, 7, 3)
    assert a[0] != a[1]
    pairs = {tuple(x["sha256"] for x in backdrops.pick(images, 7, k)) for k in range(20)}
    assert len(pairs) > 15  # each change draws anew
    assert backdrops.pick(images, 8, 3) != a


def test_an_entry_records_the_attribution():
    r = {"ImageID": "0000000000000001", "License": backdrops.CC_BY_2, "Author": "A", "AuthorProfileURL": "p", "Title": "T",
         "OriginalLandingURL": "l"}
    e = backdrops.entry(r, "ab" * 32, 10, (640, 480))
    assert e["file"] == "images/" + "ab" * 32 + ".jpg" and e["author"] == "A" and e["license"] == backdrops.CC_BY_2
    assert e["url"].endswith("/0000000000000001.jpg")
