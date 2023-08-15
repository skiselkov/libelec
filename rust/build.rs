/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
/*
 * Copyright 2023 Saso Kiselkov. All rights reserved.
 */

fn main() {
	const LACF_DIR: &str = "../nettest/libacfutils-redist-v0.37";
	println!("cargo:rustc-link-search=native=../src/build");
	println!("cargo:rustc-link-lib=static=elec");
	println!("cargo:rustc-link-search=native={}/lin64/lib", LACF_DIR);
	println!("cargo:rustc-link-lib=static=acfutils");
	println!("cargo:rustc-link-lib=static=crypto");
	println!("cargo:rustc-link-lib=static=ssl");
	println!("cargo:rustc-link-lib=static=curl");
	println!("cargo:rustc-link-lib=static=z");
}
